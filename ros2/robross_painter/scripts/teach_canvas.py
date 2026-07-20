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
  2. Run this node with the same tool offset the executor will use, and
     the intended pen preload as plane_bias_mm (see step 3):

       ros2 run robross_painter teach_canvas.py --ros-args \\
         -p tool_offset_xyz:="[0.0, 0.0, 0.12]" \\
         -p plane_bias_mm:=1.8 \\
         -p output_file:=$HOME/canvas_calibration.yaml

  3. Bring the pen tip to each corner of the paper at JUST-touch (the
     spring at free length) and record it. Use freedrive only for the
     coarse approach (hover a few mm off the corner); the i5's freedrive
     breakaway force is too high for accurate small motions. Disable
     freedrive, reactivate joint_trajectory_controller, and step in with
     the teach_nudge node (~/nudge_in, 0.2 mm steps) until the pen body
     FIRST visibly moves relative to the claw — that is the true paper
     surface — then record. The recorded point is the free-length virtual
     tip, so any spring compression at record time pushes the taught
     plane that far behind the paper; plane_bias_mm applies the drawing
     preload in software instead, at save time:

       ros2 service call /teach_canvas/record_top_left std_srvs/srv/Trigger
       ros2 service call /teach_canvas/record_top_right std_srvs/srv/Trigger
       ros2 service call /teach_canvas/record_bottom_left std_srvs/srv/Trigger
       ros2 service call /teach_canvas/record_bottom_right std_srvs/srv/Trigger

     Each record averages the pen-tip position over the last
     record_window_s seconds and is rejected if the arm moved more than
     stillness_tol_mm in that window — release the arm, let it settle,
     and call the service again. "Top-left" is the top-left of the
     artwork as it should appear on the paper (canvas x runs top-left ->
     top-right, y runs top-left -> bottom-left). Corners can be
     re-recorded at any time. bottom_right is optional: it is used only
     to validate the other three corners, and save warns when it sits
     more than corner_residual_warn_mm from the predicted rectangle
     corner.
  4. Save the calibration:

       ros2 service call /teach_canvas/save std_srvs/srv/Trigger

     The written YAML is a painting_executor parameter file; pass it to the
     paint launch as canvas_file:=<path>.
  5. Disable pendant freedrive, then return control to ROS:

       ros2 control switch_controllers \
         --activate joint_trajectory_controller --strict
"""

import collections
import math
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def average_still_samples(positions, tol_mm):
    """Average recorded pen-tip positions if the arm was still.

    Returns (mean, spread_mm) where spread_mm is the largest sample
    distance from the mean. mean is None when the spread exceeds tol_mm
    (the arm moved during the window — likely still hand-loaded).
    """
    arr = np.asarray(positions, dtype=float)
    mean = arr.mean(axis=0)
    spread_mm = float(np.linalg.norm(arr - mean, axis=1).max() * 1000.0)
    return (mean if spread_mm <= tol_mm else None), spread_mm


def compute_canvas_pose(tl, tr, bl, plane_bias_mm=0.0):
    """Canvas pose from the three touched corners.

    Returns (origin, quat_xyzw, width_m, height_m, skew_deg). origin is
    the top-left corner shifted plane_bias_mm along the canvas +z normal
    (into the wall — the executor hovers at -z, so +z is behind the
    paper). With the just-touch doctrine the corners are recorded at zero
    spring compression (the true paper surface) and the bias IS the pen
    preload: drawing compresses the spring by exactly plane_bias_mm when
    the taught plane is perfect. Raises ValueError when the corners are
    too close together to define a plane.
    """
    tl = np.asarray(tl, dtype=float)
    tr = np.asarray(tr, dtype=float)
    bl = np.asarray(bl, dtype=float)

    x_raw = tr - tl
    y_raw = bl - tl
    width_m = float(np.linalg.norm(x_raw))
    if width_m < 0.01 or np.linalg.norm(y_raw) < 0.01:
        raise ValueError("Corners are too close together, re-record them")

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
    origin = tl + zc * (plane_bias_mm / 1000.0)
    quat = matrix_to_quat(np.column_stack((xc, yc, zc)))
    return origin, quat, width_m, height_m, skew_deg


def rectangle_residual_mm(tl, tr, bl, br):
    """Distance from the measured bottom-right corner to where the other
    three corners predict it (tr + bl - tl), in millimeters."""
    predicted = np.asarray(tr, dtype=float) + np.asarray(bl, dtype=float) \
        - np.asarray(tl, dtype=float)
    return float(np.linalg.norm(np.asarray(br, dtype=float) - predicted) * 1000.0)


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
    # Recorded only to validate the required three; never used for the pose.
    OPTIONAL_CORNERS = ("bottom_right",)
    SAMPLE_RATE_HZ = 50.0

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
        # Each record averages the pen tip over this window and rejects the
        # sample if the arm moved more than stillness_tol_mm within it.
        self.declare_parameter("record_window_s", 1.0)
        self.declare_parameter("stillness_tol_mm", 0.5)
        self.declare_parameter("min_record_samples", 10)
        self.declare_parameter("corner_residual_warn_mm", 2.0)
        # Pen preload: the saved plane is shifted this far along the canvas
        # normal INTO the wall. Use with the just-touch doctrine (record
        # each corner at first visible spring movement, zero compression);
        # keep 0.0 when corners are recorded the old way, with the spring
        # already ~half compressed, or the preload doubles up.
        self.declare_parameter("plane_bias_mm", 0.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.tool_offset = np.array(self.get_parameter("tool_offset_xyz").value)
        self.record_window_s = float(self.get_parameter("record_window_s").value)
        self.stillness_tol_mm = float(self.get_parameter("stillness_tol_mm").value)
        self.min_record_samples = int(self.get_parameter("min_record_samples").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.points = {}
        # (monotonic time, pen-tip position) history, twice the record window.
        self.samples = collections.deque(
            maxlen=max(2, int(2 * self.record_window_s * self.SAMPLE_RATE_HZ))
        )
        self._tf_wait_logged = False
        self.create_timer(1.0 / self.SAMPLE_RATE_HZ, self._sample_pen_tip)

        for corner in self.CORNERS + self.OPTIONAL_CORNERS:
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
            "~/record_bottom_left (plus optional ~/record_bottom_right for "
            "validation), then ~/save. Hands off the arm while recording."
        )

    def _sample_pen_tip(self):
        try:
            p = self.pen_tip_in_base()
        except Exception:  # TF not available yet / frame missing
            if not self._tf_wait_logged:
                self.get_logger().info(
                    f"Waiting for TF {self.base_frame} -> {self.ee_frame}..."
                )
                self._tf_wait_logged = True
            return
        self.samples.append((time.monotonic(), p))

    def pen_tip_in_base(self):
        tf = self.tf_buffer.lookup_transform(
            self.base_frame, self.ee_frame, rclpy.time.Time()
        )
        t = tf.transform.translation
        q = tf.transform.rotation
        rot = quat_to_matrix(q.x, q.y, q.z, q.w)
        return np.array([t.x, t.y, t.z]) + rot @ self.tool_offset

    def record(self, corner, res):
        cutoff = time.monotonic() - self.record_window_s
        window = [p for t, p in self.samples if t >= cutoff]
        if len(window) < self.min_record_samples:
            res.success = False
            res.message = (
                f"Only {len(window)} pen-tip samples in the last "
                f"{self.record_window_s:.1f} s (need "
                f"{self.min_record_samples}); TF may not be streaming yet — "
                "wait a moment and re-record"
            )
            return res
        p, spread_mm = average_still_samples(window, self.stillness_tol_mm)
        if p is None:
            res.success = False
            res.message = (
                f"Arm moved {spread_mm:.2f} mm during the last "
                f"{self.record_window_s:.1f} s (tolerance "
                f"{self.stillness_tol_mm:.2f} mm). Release the arm, let it "
                "settle, and re-record"
            )
            self.get_logger().warn(res.message)
            return res
        self.points[corner] = p
        missing = [c for c in self.CORNERS if c not in self.points]
        res.success = True
        res.message = (
            f"{corner} = [{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}] m "
            f"(mean of {len(window)} samples, spread {spread_mm:.2f} mm)"
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
        bias_mm = float(self.get_parameter("plane_bias_mm").value)

        try:
            origin, (qx, qy, qz, qw), width_m, height_m, skew_deg = \
                compute_canvas_pose(tl, tr, bl, bias_mm)
        except ValueError as exc:
            res.success = False
            res.message = str(exc)
            return res

        warnings = []
        # 3.8 mm is the pen spring's full travel: a negative bias draws in
        # the air, a bias past full travel bottoms the spring out.
        if bias_mm < 0.0 or bias_mm > 3.8:
            warnings.append(
                f"plane_bias_mm {bias_mm:.1f} is outside the pen spring's "
                "0-3.8 mm travel — check the sign/value"
            )
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
        residual_mm = None
        if "bottom_right" in self.points:
            residual_mm = rectangle_residual_mm(
                tl, tr, bl, self.points["bottom_right"]
            )
            warn_mm = self.get_parameter("corner_residual_warn_mm").value
            if residual_mm > warn_mm:
                warnings.append(
                    f"bottom-right corner is {residual_mm:.1f} mm from the "
                    f"position the other corners predict (tolerance "
                    f"{warn_mm:.1f} mm), re-teach"
                )
        for w in warnings:
            self.get_logger().warn(w)

        out = self.get_parameter("output_file").value
        data = {
            "painting_executor": {
                "ros__parameters": {
                    "canvas_origin_xyz": [round(float(v), 6) for v in origin],
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
            f"# plane_bias_mm: {bias_mm} (origin sits this far behind the "
            "raw top_left, along the canvas normal into the wall)\n"
            f"# top_left:     {tl.tolist()}\n"
            f"# top_right:    {tr.tolist()}\n"
            f"# bottom_left:  {bl.tolist()}\n"
            + (
                f"# bottom_right: "
                f"{self.points['bottom_right'].tolist()} "
                f"(validation residual {residual_mm:.2f} mm)\n"
                if residual_mm is not None else ""
            )
            + "# Pass to the executor as: canvas_file:=<this file>\n"
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
            f"{height_m * 1000:.1f} mm, plane bias {bias_mm:.1f} mm"
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
