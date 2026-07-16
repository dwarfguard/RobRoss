#!/usr/bin/env python3
"""Teach the canvas (paper) pose by touching its corners with the pen tip.

Works for paper taped to a wall, lying on a table, or on a slanted stand:
the three touched corners define the full 3D canvas plane.

Workflow:
  1. Start the Aubo driver (real hardware) so TF base_link -> ee_link is
     published. Release servo control before enabling pendant freedrive:

       ros2 control switch_controllers \
         --deactivate joint_trajectory_controller --strict

     Confirm joint_trajectory_controller is inactive and
     joint_state_broadcaster remains active, then enable freedrive.
  2. Run this node with the same tool offset the executor will use:

       ros2 run robross_painter teach_canvas.py --ros-args \\
         -p tool_offset_xyz:="[0.0, 0.0, 0.12]" \\
         -p output_file:=$HOME/canvas_calibration.yaml

  3. Touch the pen tip to each corner of the paper and record it:

       ros2 service call /teach_canvas/record_top_left std_srvs/srv/Trigger
       ros2 service call /teach_canvas/record_top_right std_srvs/srv/Trigger
       ros2 service call /teach_canvas/record_bottom_left std_srvs/srv/Trigger

     "Top-left" is the top-left of the artwork as it should appear on the
     paper (canvas x runs top-left -> top-right, y runs top-left ->
     bottom-left). Corners can be re-recorded at any time.
  4. Save the calibration:

       ros2 service call /teach_canvas/save std_srvs/srv/Trigger

     The written YAML is a painting_executor parameter file; pass it to the
     paint launch as canvas_file:=<path>.
  5. Disable pendant freedrive, then return control to ROS:

       ros2 control switch_controllers \
         --activate joint_trajectory_controller --strict
"""

import math

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat(m):
    # Standard Shepperd method; returns (x, y, z, w).
    t = np.trace(m)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        return (
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
            0.25 * s,
        )
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(m[i, i] - m[j, j] - m[k, k] + 1.0) * 2.0
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    q[3] = (m[k, j] - m[j, k]) / s
    return tuple(q)


class TeachCanvas(Node):
    CORNERS = ("top_left", "top_right", "bottom_left")

    def __init__(self):
        super().__init__("teach_canvas")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "ee_link")
        # Pen tip position in the ee frame (the claw's grip geometry).
        # Must match the executor's tool_offset_xyz.
        self.declare_parameter("tool_offset_xyz", [0.0, 0.0, 0.0])
        # Expected paper size, used only for a sanity warning. 0 disables.
        self.declare_parameter("canvas_width_mm", 210.0)
        self.declare_parameter("canvas_height_mm", 297.0)
        self.declare_parameter("output_file", "canvas_calibration.yaml")

        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.tool_offset = np.array(self.get_parameter("tool_offset_xyz").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.points = {}

        for corner in self.CORNERS:
            self.create_service(
                Trigger,
                f"~/record_{corner}",
                lambda req, res, c=corner: self.record(c, res),
            )
        self.create_service(Trigger, "~/save", self.save)

        self.get_logger().info(
            "Teach mode: verify joint_trajectory_controller is inactive, "
            "enable pendant freedrive, touch the pen tip to each paper corner "
            "and call ~/record_top_left, ~/record_top_right, "
            "~/record_bottom_left, then ~/save."
        )

    def pen_tip_in_base(self):
        tf = self.tf_buffer.lookup_transform(
            self.base_frame, self.ee_frame, rclpy.time.Time()
        )
        t = tf.transform.translation
        q = tf.transform.rotation
        rot = quat_to_matrix(q.x, q.y, q.z, q.w)
        return np.array([t.x, t.y, t.z]) + rot @ self.tool_offset

    def record(self, corner, res):
        try:
            p = self.pen_tip_in_base()
        except Exception as exc:  # TF not available yet / frame missing
            res.success = False
            res.message = f"TF lookup failed: {exc}"
            return res
        self.points[corner] = p
        missing = [c for c in self.CORNERS if c not in self.points]
        res.success = True
        res.message = (
            f"{corner} = [{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}] m"
            + (f"; still missing: {', '.join(missing)}" if missing else
               "; all corners recorded, call ~/save")
        )
        self.get_logger().info(res.message)
        return res

    def save(self, _req, res):
        missing = [c for c in self.CORNERS if c not in self.points]
        if missing:
            res.success = False
            res.message = f"Missing corners: {', '.join(missing)}"
            return res

        tl = self.points["top_left"]
        tr = self.points["top_right"]
        bl = self.points["bottom_left"]

        x_raw = tr - tl
        y_raw = bl - tl
        width_m = float(np.linalg.norm(x_raw))
        if width_m < 0.01 or np.linalg.norm(y_raw) < 0.01:
            res.success = False
            res.message = "Corners are too close together, re-record them"
            return res

        xc = x_raw / width_m
        y_proj = y_raw - np.dot(y_raw, xc) * xc
        height_m = float(np.linalg.norm(y_proj))
        yc = y_proj / height_m
        zc = np.cross(xc, yc)
        skew_deg = math.degrees(
            math.acos(
                np.clip(
                    np.dot(y_raw / np.linalg.norm(y_raw), yc), -1.0, 1.0
                )
            )
        )

        qx, qy, qz, qw = matrix_to_quat(np.column_stack((xc, yc, zc)))

        warnings = []
        if skew_deg > 2.0:
            warnings.append(
                f"corners are {skew_deg:.1f} deg off square, re-teach if "
                "the paper is not skewed"
            )
        exp_w = self.get_parameter("canvas_width_mm").value
        exp_h = self.get_parameter("canvas_height_mm").value
        if exp_w > 0 and abs(width_m * 1000.0 - exp_w) > 5.0:
            warnings.append(
                f"measured width {width_m * 1000:.1f} mm vs expected "
                f"{exp_w:.0f} mm"
            )
        if exp_h > 0 and abs(height_m * 1000.0 - exp_h) > 5.0:
            warnings.append(
                f"measured height {height_m * 1000:.1f} mm vs expected "
                f"{exp_h:.0f} mm"
            )
        for w in warnings:
            self.get_logger().warn(w)

        out = self.get_parameter("output_file").value
        data = {
            "painting_executor": {
                "ros__parameters": {
                    "canvas_origin_xyz": [round(float(v), 6) for v in tl],
                    "canvas_quat_xyzw": [
                        round(float(v), 6) for v in (qx, qy, qz, qw)
                    ],
                }
            }
        }
        header = (
            "# Canvas calibration written by teach_canvas.py.\n"
            f"# base frame: {self.base_frame}, ee frame: {self.ee_frame}, "
            f"tool_offset_xyz: {self.tool_offset.tolist()}\n"
            f"# measured paper: {width_m * 1000:.1f} x "
            f"{height_m * 1000:.1f} mm, corner skew {skew_deg:.2f} deg\n"
            f"# top_left:     {tl.tolist()}\n"
            f"# top_right:    {tr.tolist()}\n"
            f"# bottom_left:  {bl.tolist()}\n"
            "# Pass to the executor as: canvas_file:=<this file>\n"
        )
        try:
            with open(out, "w") as f:
                f.write(header)
                yaml.safe_dump(data, f, default_flow_style=None)
        except OSError as exc:
            res.success = False
            res.message = f"Cannot write {out}: {exc}"
            return res

        res.success = True
        res.message = (
            f"Saved {out} (paper {width_m * 1000:.1f} x "
            f"{height_m * 1000:.1f} mm"
            + (f"; WARNINGS: {'; '.join(warnings)})" if warnings else ")")
        )
        self.get_logger().info(res.message)
        return res


def main():
    rclpy.init()
    node = TeachCanvas()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
