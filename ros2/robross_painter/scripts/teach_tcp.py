#!/usr/bin/env python3
"""Calibrate the pen-tip tool offset (TCP) by touching a fixed sharp pin.

Replaces the hand-measured tool_offset_xyz / tool_offset_rpy with a measured
pivot ("N-point") calibration: clamp a sharp pin in space, touch the pen tip to
that single point from several very different wrist orientations, and this node
least-squares solves the tip location in the ee_link frame — the same frame the
painting_executor and teach_canvas use, so the result drops straight into the
config profiles with no frame conversion.

Two parts:

  A. Tip position (tool_offset_xyz). Touch the pin tip from >= min_tip_poses
     WIDELY VARIED wrist orientations (a large attitude spread — near-identical
     orientations make the solve ill-conditioned). For each touch the node
     records the still-averaged ee_link position p_i and its rotation R_i and
     solves  p_i + R_i * t = P  for the tip t (in ee_link) and pin point P.
     The RMS of |(p_i + R_i t) - P| over the touches is the tip scatter — the
     built-in accuracy readout. Add touches until it is small (< residual_warn_mm).

  B. Pen axis (tool_offset_rpy). Two options:
       * VERTICAL (primary, needs only a small bubble level): with the tip on
         the pin, orient the pen plumb (a claw/barrel flat against the level)
         and call ~/record_axis_vertical a few times. axis = R^T * z_base.
       * TWO-PIVOT (higher accuracy if you have a second identifiable point on
         the pen centerline): run the pivot a second time via ~/record_axis_point
         touching that second point; axis = normalize(tip - second_point).
         Beware the radius-offset error if you touch the side of a bare barrel.
     If neither is recorded, tool_offset_rpy stays [0, 0, 0] (pen assumed
     parallel to ee +Z, as before).

Workflow (mirrors teach_canvas.py — reuse the same freedrive + teach_nudge
just-touch approach):
  1. Start the Aubo driver (real hardware) so TF base_link -> ee_link streams.
  2. ros2 run robross_painter teach_tcp.py --ros-args \\
       -p output_file:=$HOME/tcp_calibration.yaml
     (teach_tcp needs no tool offset and no MoveGroup — it only reads TF.)
  3. Per touch: pendant freedrive to hover near the pin, disable freedrive,
     reactivate joint_trajectory_controller, step in with teach_nudge
     (~/nudge_in, 0.2 mm) to just-touch, then:
       ros2 service call /teach_tcp/record_tip std_srvs/srv/Trigger
     Reorient the wrist a lot between touches. Check progress any time:
       ros2 service call /teach_tcp/solve std_srvs/srv/Trigger
  4. (Optional axis) touch + plumb the pen, then:
       ros2 service call /teach_tcp/record_axis_vertical std_srvs/srv/Trigger
  5. ros2 service call /teach_tcp/save std_srvs/srv/Trigger

After saving: copy tool_offset_xyz / tool_offset_rpy into ALL FOUR config
profiles (hardware_a4.yaml, rviz_wall_a4.yaml, rviz_taught_a4.yaml,
demo_v1_rviz.yaml) so they stay identical, re-pick tool_spin_deg by eye for
claw/cable clearance, and RE-TEACH THE CANVAS (any existing canvas_calibration
was recorded with the old offset and is now stale).
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

# ---------------------------------------------------------------------------
# Helpers duplicated from teach_canvas.py (the two teach nodes install as bare
# executables, not an importable package; the repo already accepts small
# byte-for-byte duplication, cf. the path_validation.py copies).
# ---------------------------------------------------------------------------


def average_still_samples(positions, tol_mm):
    """Average recorded positions if the arm was still.

    Returns (mean, spread_mm) where spread_mm is the largest sample distance
    from the mean. mean is None when the spread exceeds tol_mm (the arm moved
    during the window — likely still hand-loaded).
    """
    arr = np.asarray(positions, dtype=float)
    mean = arr.mean(axis=0)
    spread_mm = float(np.linalg.norm(arr - mean, axis=1).max() * 1000.0)
    return (mean if spread_mm <= tol_mm else None), spread_mm


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


# ---------------------------------------------------------------------------
# TCP calibration math (new; unit-tested in test/test_teach_tcp_math.py).
# ---------------------------------------------------------------------------


def solve_pivot(poses):
    """Least-squares pivot solve for a fixed tool point touched many ways.

    poses is a list of (p, R): p the ee_link origin in base (3,), R the ee_link
    rotation in base (3x3). All touches contact the SAME fixed point, so for the
    unknown tool point t (in ee_link) and world point P:  p_i + R_i t = P.
    Stacking [R_i | -I] [t; P] = -p_i and solving gives t and P.

    Returns (t, P, rms_mm, rank, cond):
      t       tool point in ee_link (3,) — the tool_offset_xyz
      P       fixed world point in base (3,)
      rms_mm  RMS of the per-touch tip scatter |(p_i + R_i t) - P|, in mm
      rank    rank of the stacked system (6 when well-posed)
      cond    condition number (blows up when the orientations barely differ)
    """
    n = len(poses)
    a = np.zeros((3 * n, 6))
    b = np.zeros(3 * n)
    eye = np.eye(3)
    for i, (p, r) in enumerate(poses):
        a[3 * i:3 * i + 3, 0:3] = r
        a[3 * i:3 * i + 3, 3:6] = -eye
        b[3 * i:3 * i + 3] = -np.asarray(p, dtype=float)
    sol, _res, rank, sv = np.linalg.lstsq(a, b, rcond=None)
    t = sol[:3]
    world = sol[3:]
    tips = np.array([np.asarray(p, dtype=float) + r @ t for p, r in poses])
    rms_mm = float(np.sqrt(np.mean(np.sum((tips - world) ** 2, axis=1))) * 1000.0)
    cond = float(sv[0] / sv[-1]) if sv[-1] > 0.0 else math.inf
    return t, world, rms_mm, int(rank), cond


def minimal_rotation_z_to(v):
    """Rotation matrix taking ee +Z onto unit vector v with no roll about it."""
    v = np.asarray(v, dtype=float)
    v = v / np.linalg.norm(v)
    z = np.array([0.0, 0.0, 1.0])
    c = float(np.dot(z, v))
    if c > 1.0 - 1e-12:
        return np.eye(3)
    if c < -1.0 + 1e-12:
        # Antiparallel: 180 deg about any axis perpendicular to z (use x).
        return np.diag([1.0, -1.0, -1.0])
    axis = np.cross(z, v)
    s = np.linalg.norm(axis)
    axis /= s
    k = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ]
    )
    # Rodrigues: R = I + sin(theta) K + (1 - cos(theta)) K^2, sin=s, cos=c.
    return np.eye(3) + s * k + (1.0 - c) * (k @ k)


def matrix_to_rpy(m):
    """Extract tf2 setRPY (roll, pitch, yaw) from a rotation matrix.

    tf2's setRPY builds R = Rz(yaw) Ry(pitch) Rx(roll), so this is the standard
    ZYX-intrinsic / XYZ-fixed extraction with a gimbal-lock fallback.
    """
    if abs(m[2, 0]) < 1.0 - 1e-9:
        pitch = math.atan2(-m[2, 0], math.hypot(m[0, 0], m[1, 0]))
        yaw = math.atan2(m[1, 0], m[0, 0])
        roll = math.atan2(m[2, 1], m[2, 2])
    else:  # pitch = +-90 deg: roll and yaw are coupled, fold into roll.
        yaw = 0.0
        if m[2, 0] <= -1.0 + 1e-9:
            pitch = math.pi / 2.0
            roll = math.atan2(m[0, 1], m[0, 2])
        else:
            pitch = -math.pi / 2.0
            roll = math.atan2(-m[0, 1], -m[0, 2])
    return [roll, pitch, yaw]


def rpy_from_axis(axis_ee):
    """tool_offset_rpy for a pen pointing along axis_ee (unit, in ee_link)."""
    return matrix_to_rpy(minimal_rotation_z_to(axis_ee))


class TeachTcp(Node):
    SAMPLE_RATE_HZ = 50.0
    Z_BASE = np.array([0.0, 0.0, 1.0])

    def __init__(self):
        super().__init__("teach_tcp")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "ee_link")
        self.declare_parameter("output_file", "tcp_calibration.yaml")
        # Each record averages the ee pose over this window and rejects the
        # sample if the arm moved more than stillness_tol_mm within it.
        self.declare_parameter("record_window_s", 1.0)
        self.declare_parameter("stillness_tol_mm", 0.5)
        self.declare_parameter("min_record_samples", 10)
        # Minimum tip touches before a solve is trusted, and the tip-scatter
        # RMS above which the calibration is flagged as too loose.
        self.declare_parameter("min_tip_poses", 4)
        self.declare_parameter("residual_warn_mm", 0.7)
        # Condition number above which the touch orientations are too similar.
        # Empirically cond ~= 187 / (total wrist spread in degrees), so a good
        # calibration (30-90 deg spread) sits at 2-6; 100 flags under ~2 deg of
        # spread — reorient the wrist far more than that between touches.
        self.declare_parameter("cond_warn", 100.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.record_window_s = float(self.get_parameter("record_window_s").value)
        self.stillness_tol_mm = float(self.get_parameter("stillness_tol_mm").value)
        self.min_record_samples = int(self.get_parameter("min_record_samples").value)
        self.min_tip_poses = int(self.get_parameter("min_tip_poses").value)
        self.residual_warn_mm = float(self.get_parameter("residual_warn_mm").value)
        self.cond_warn = float(self.get_parameter("cond_warn").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Touch buckets: (position, rotation) pairs in base.
        self.tip_poses = []
        self.axis_point_poses = []
        # Pen-axis-in-ee samples from the plumb (vertical) method.
        self.axis_vertical = []
        # (monotonic, ee position, ee rotation) history, twice the window.
        self.samples = collections.deque(
            maxlen=max(2, int(2 * self.record_window_s * self.SAMPLE_RATE_HZ))
        )
        self._tf_wait_logged = False
        self.create_timer(1.0 / self.SAMPLE_RATE_HZ, self._sample_ee)

        self.create_service(Trigger, "~/record_tip", self._srv_record_tip)
        self.create_service(
            Trigger, "~/record_axis_point", self._srv_record_axis_point
        )
        self.create_service(
            Trigger, "~/record_axis_vertical", self._srv_record_axis_vertical
        )
        self.create_service(Trigger, "~/clear", self._srv_clear)
        self.create_service(Trigger, "~/solve", self._srv_solve)
        self.create_service(Trigger, "~/save", self._srv_save)

        self.get_logger().info(
            "Teach TCP: touch the fixed pin with the pen tip from >= "
            f"{self.min_tip_poses} widely varied wrist orientations, calling "
            "~/record_tip each time; ~/solve to check the tip scatter, "
            "~/record_axis_vertical (pen plumb) for the pen axis, then ~/save. "
            "Hands off the arm while recording."
        )

    # --- sampling -----------------------------------------------------------

    def _sample_ee(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time()
            )
        except Exception:  # TF not available yet / frame missing
            if not self._tf_wait_logged:
                self.get_logger().info(
                    f"Waiting for TF {self.base_frame} -> {self.ee_frame}..."
                )
                self._tf_wait_logged = True
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        self.samples.append(
            (
                time.monotonic(),
                np.array([t.x, t.y, t.z]),
                quat_to_matrix(q.x, q.y, q.z, q.w),
            )
        )

    def _still_pose(self):
        """Still-averaged (position, rotation) over the record window.

        Returns (p, R, message). p/R are None when there are too few samples or
        the arm moved; message explains why (or is empty on success).
        """
        cutoff = time.monotonic() - self.record_window_s
        window = [(p, r) for tm, p, r in self.samples if tm >= cutoff]
        if len(window) < self.min_record_samples:
            return None, None, (
                f"Only {len(window)} ee samples in the last "
                f"{self.record_window_s:.1f} s (need {self.min_record_samples}); "
                "TF may not be streaming yet — wait a moment and re-record"
            )
        mean, spread_mm = average_still_samples(
            [p for p, _ in window], self.stillness_tol_mm
        )
        if mean is None:
            return None, None, (
                f"Arm moved {spread_mm:.2f} mm during the last "
                f"{self.record_window_s:.1f} s (tolerance "
                f"{self.stillness_tol_mm:.2f} mm). Release the arm, let it "
                "settle, and re-record"
            )
        # Arm is still, so the latest rotation is representative.
        return mean, window[-1][1], ""

    # --- record services ----------------------------------------------------

    def _srv_record_tip(self, _req, res):
        p, r, msg = self._still_pose()
        if p is None:
            res.success, res.message = False, msg
            self.get_logger().warn(msg)
            return res
        self.tip_poses.append((p, r))
        res.success = True
        res.message = f"Recorded tip touch #{len(self.tip_poses)}"
        if len(self.tip_poses) >= self.min_tip_poses:
            _t, _w, rms_mm, _rank, cond = solve_pivot(self.tip_poses)
            res.message += (
                f"; tip scatter {rms_mm:.2f} mm (cond {cond:.0f})"
            )
        else:
            res.message += (
                f"; need {self.min_tip_poses - len(self.tip_poses)} more before "
                "a solve"
            )
        self.get_logger().info(res.message)
        return res

    def _srv_record_axis_point(self, _req, res):
        p, r, msg = self._still_pose()
        if p is None:
            res.success, res.message = False, msg
            self.get_logger().warn(msg)
            return res
        self.axis_point_poses.append((p, r))
        res.success = True
        res.message = (
            f"Recorded axis second-point touch #{len(self.axis_point_poses)}"
        )
        self.get_logger().info(res.message)
        return res

    def _srv_record_axis_vertical(self, _req, res):
        p, r, msg = self._still_pose()
        if p is None:
            res.success, res.message = False, msg
            self.get_logger().warn(msg)
            return res
        # Pen held plumb => pen axis is base +Z, expressed in ee coordinates.
        axis_ee = r.T @ self.Z_BASE
        self.axis_vertical.append(axis_ee)
        res.success = True
        res.message = (
            f"Recorded plumb axis sample #{len(self.axis_vertical)}"
        )
        self.get_logger().info(res.message)
        return res

    def _srv_clear(self, _req, res):
        self.tip_poses.clear()
        self.axis_point_poses.clear()
        self.axis_vertical.clear()
        res.success = True
        res.message = "Cleared all recorded touches"
        self.get_logger().info(res.message)
        return res

    # --- solve / save -------------------------------------------------------

    def _compute(self):
        """Solve the current data. Returns (result_dict, warnings) or raises
        ValueError when there are too few tip touches."""
        if len(self.tip_poses) < self.min_tip_poses:
            raise ValueError(
                f"Only {len(self.tip_poses)} tip touches (need "
                f"{self.min_tip_poses}); record more with ~/record_tip"
            )
        warnings = []
        t, world, rms_mm, rank, cond = solve_pivot(self.tip_poses)
        if rank < 6 or cond > self.cond_warn:
            warnings.append(
                f"tip touches are near-degenerate (rank {rank}, cond {cond:.0f}) "
                "— vary the wrist orientation more between touches"
            )
        if rms_mm > self.residual_warn_mm:
            warnings.append(
                f"tip scatter {rms_mm:.2f} mm exceeds {self.residual_warn_mm:.2f} "
                "mm — re-touch just-touch and add more varied poses"
            )

        axis_ee = None
        axis_method = "none (pen assumed parallel to ee +Z)"
        axis_rms_mm = None
        if len(self.axis_point_poses) >= self.min_tip_poses:
            second, _w2, axis_rms_mm, arank, acond = solve_pivot(
                self.axis_point_poses
            )
            axis_ee = t - second
            axis_method = "two-pivot (tip - second point)"
            if arank < 6 or acond > self.cond_warn:
                warnings.append(
                    "axis second-point touches are near-degenerate — vary the "
                    "wrist more or use the plumb method"
                )
        elif self.axis_vertical:
            axis_ee = np.mean(np.asarray(self.axis_vertical), axis=0)
            axis_method = f"plumb ({len(self.axis_vertical)} samples)"

        if axis_ee is not None:
            norm = np.linalg.norm(axis_ee)
            if norm < 1e-6:
                warnings.append(
                    "axis is degenerate (tip and second point coincide) — "
                    "keeping tool_offset_rpy [0, 0, 0]"
                )
                axis_ee = None
        if axis_ee is not None:
            axis_ee = axis_ee / np.linalg.norm(axis_ee)
            # Keep the sign continuous with the current [0,0,0] convention.
            if float(np.dot(axis_ee, self.Z_BASE)) < 0.0:
                axis_ee = -axis_ee
            rpy = rpy_from_axis(axis_ee)
            tilt_deg = math.degrees(
                math.acos(np.clip(float(np.dot(axis_ee, self.Z_BASE)), -1.0, 1.0))
            )
        else:
            rpy = [0.0, 0.0, 0.0]
            tilt_deg = 0.0

        result = {
            "tool_offset_xyz": [round(float(v), 6) for v in t],
            "tool_offset_rpy": [round(float(v), 6) for v in rpy],
            "world_point": [round(float(v), 6) for v in world],
            "n_tip": len(self.tip_poses),
            "tip_rms_mm": rms_mm,
            "tip_cond": cond,
            "axis_method": axis_method,
            "axis_rms_mm": axis_rms_mm,
            "axis_tilt_deg": tilt_deg,
        }
        return result, warnings

    def _srv_solve(self, _req, res):
        try:
            result, warnings = self._compute()
        except ValueError as exc:
            res.success, res.message = False, str(exc)
            self.get_logger().warn(res.message)
            return res
        for w in warnings:
            self.get_logger().warn(w)
        res.success = True
        res.message = (
            f"tool_offset_xyz={result['tool_offset_xyz']} "
            f"tool_offset_rpy={result['tool_offset_rpy']} "
            f"(tip scatter {result['tip_rms_mm']:.2f} mm over "
            f"{result['n_tip']} touches, axis {result['axis_method']}, "
            f"tilt {result['axis_tilt_deg']:.2f} deg)"
            + (f"; WARNINGS: {'; '.join(warnings)}" if warnings else "")
        )
        self.get_logger().info(res.message)
        return res

    def _srv_save(self, _req, res):
        try:
            result, warnings = self._compute()
        except ValueError as exc:
            res.success, res.message = False, str(exc)
            self.get_logger().warn(res.message)
            return res
        for w in warnings:
            self.get_logger().warn(w)

        out = self.get_parameter("output_file").value
        data = {
            "painting_executor": {
                "ros__parameters": {
                    "tool_offset_xyz": result["tool_offset_xyz"],
                    "tool_offset_rpy": result["tool_offset_rpy"],
                }
            }
        }
        axis_rms = (
            f"{result['axis_rms_mm']:.2f} mm"
            if result["axis_rms_mm"] is not None else "n/a"
        )
        header = (
            "# Pen-tip TCP calibration written by teach_tcp.py.\n"
            f"# base frame: {self.base_frame}, ee frame: {self.ee_frame}\n"
            f"# tip: {result['n_tip']} touches, scatter RMS "
            f"{result['tip_rms_mm']:.3f} mm, cond {result['tip_cond']:.0f}\n"
            f"# pin world point: {result['world_point']} m\n"
            f"# pen axis: {result['axis_method']}, tilt {result['axis_tilt_deg']:.2f} "
            f"deg vs ee +Z, axis fit {axis_rms}\n"
            "# ACTIONS after applying:\n"
            "#   1. Copy tool_offset_xyz/rpy into ALL FOUR profiles (hardware_a4,\n"
            "#      rviz_wall_a4, rviz_taught_a4, demo_v1_rviz) — keep identical.\n"
            "#   2. Re-pick tool_spin_deg by eye for claw/cable clearance.\n"
            "#   3. RE-TEACH the canvas — the old canvas_calibration used the old\n"
            "#      offset and is now stale.\n"
            + "".join(f"# WARNING: {w}\n" for w in warnings)
        )
        try:
            with open(out, "w") as f:
                f.write(header)
                yaml.safe_dump(data, f, default_flow_style=None)
        except OSError as exc:
            res.success, res.message = False, f"Cannot write {out}: {exc}"
            return res

        res.success = True
        res.message = (
            f"Saved {out}: tool_offset_xyz={result['tool_offset_xyz']}, "
            f"tool_offset_rpy={result['tool_offset_rpy']} (tip scatter "
            f"{result['tip_rms_mm']:.2f} mm)"
            + (f"; WARNINGS: {'; '.join(warnings)}" if warnings else "")
        )
        self.get_logger().info(res.message)
        return res


def main():
    rclpy.init()
    node = TeachTcp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
