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
     re-recorded at any time. All FOUR corners are required: the plane
     normal is the least-squares best fit through every touched point, so
     the fourth corner averages out per-corner touch noise instead of
     just validating the other three.
  3b. Optionally record interior sample points the same way, at just-touch,
     spread across the paper (a ~3x3 grid: center, mid-edges, quarter
     points — ~5-9 points):

       ros2 service call /teach_canvas/record_sample std_srvs/srv/Trigger

     These are fit into a smooth Z-correction surface recorded in the
     saved YAML as a flatness DIAGNOSTIC only — the executor does NOT
     apply it during motion (the tracking remediation plan,
     docs/aubo-painting-tracking-remediation-plan.md Section 4, forbids
     position-dependent Z compensation). The fit measures the
     reach-dependent, NON-planar contact error that a single flat plane
     cannot represent. save reports the out-of-plane error before and
     after the fitted surface and refuses when too much remains.
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


def fit_plane_normal(points, ref_normal):
    """Unit normal of the least-squares best-fit plane through >=3 points.

    Sign-aligned to ref_normal so it keeps pointing into the paper. Unlike
    cross(xc, yc) from two corner edges, this uses every touched point, so a
    single noisy corner no longer tips the whole plane.
    """
    pts = np.asarray(points, dtype=float)
    centroid = pts.mean(axis=0)
    _u, _s, vh = np.linalg.svd(pts - centroid)
    normal = vh[-1]
    if np.dot(normal, ref_normal) < 0.0:
        normal = -normal
    return normal / np.linalg.norm(normal)


# Correction surface basis: [1, x, y, x*y, x^2, y^2], x/y in millimeters.
_CORR_TERMS = 6


def _corr_design(x_mm, y_mm):
    x = np.asarray(x_mm, dtype=float)
    y = np.asarray(y_mm, dtype=float)
    return np.column_stack([np.ones_like(x), x, y, x * y, x * x, y * y])


def fit_z_correction(x_mm, y_mm, resid_mm):
    """Least-squares coefficients of the correction surface
    z_corr(x, y) = a + b*x + c*y + d*x*y + e*x^2 + f*y^2 (mm), fit to the
    per-point out-of-plane residuals.

    Uses as many terms as the point count supports (>=6 quadratic, >=3 linear,
    else flat) and zero-pads the rest, so a sparse teach still yields a valid,
    non-overfit surface. Returns the full six coefficients.
    """
    n = np.asarray(x_mm, dtype=float).shape[0]
    if n >= _CORR_TERMS:
        ncols = _CORR_TERMS
    elif n >= 3:
        ncols = 3  # linear residual (tilt) only
    else:
        return [0.0] * _CORR_TERMS
    design = _corr_design(x_mm, y_mm)[:, :ncols]
    coeffs, *_ = np.linalg.lstsq(
        design, np.asarray(resid_mm, dtype=float), rcond=None
    )
    return [float(v) for v in coeffs] + [0.0] * (_CORR_TERMS - ncols)


def evaluate_z_correction(coeffs, x_mm, y_mm):
    """Evaluate the correction surface (mm) at canvas (x_mm, y_mm)."""
    return float(_corr_design([x_mm], [y_mm])[0] @ np.asarray(coeffs, float))


def compute_canvas_calibration(tl, tr, bl, br, samples=(), plane_bias_mm=0.0):
    """Full canvas calibration from four touched corners plus interior samples.

    Returns (origin, quat_xyzw, width_m, height_m, corr_coeffs,
    max_resid_before_mm, max_resid_after_mm).

    The in-plane frame (canvas x right, y down) still comes from the corners,
    but the plane NORMAL is the least-squares best fit through every touched
    point (corners + samples). corr_coeffs is a quadratic surface (mm) fitted
    to the out-of-plane residuals as a flatness diagnostic; the executor does
    NOT apply it during motion (the remediation plan forbids position-dependent
    Z compensation). origin is the top-left
    corner shifted plane_bias_mm along the fitted +z normal (the pen preload).
    max_resid_before_mm is the worst out-of-plane error the flat model would
    leave; max_resid_after_mm is what would remain if the surface were applied.
    Raises ValueError when the corners are too close to define a plane.
    """
    tl = np.asarray(tl, dtype=float)
    tr = np.asarray(tr, dtype=float)
    bl = np.asarray(bl, dtype=float)
    br = np.asarray(br, dtype=float)

    x_raw = tr - tl
    y_raw = bl - tl
    width_m = float(np.linalg.norm(x_raw))
    if width_m < 0.01 or np.linalg.norm(y_raw) < 0.01:
        raise ValueError("Corners are too close together, re-record them")

    xc0 = x_raw / width_m
    y_proj0 = y_raw - np.dot(y_raw, xc0) * xc0
    yc0 = y_proj0 / np.linalg.norm(y_proj0)
    n_ref = np.cross(xc0, yc0)  # rough into-paper normal from the corners

    all_pts = np.vstack(
        [tl, tr, bl, br] + [np.asarray(s, dtype=float) for s in samples]
    )
    zc = fit_plane_normal(all_pts, n_ref)

    # Re-orthogonalize the in-plane axes into the fitted plane.
    xc = xc0 - np.dot(xc0, zc) * zc
    xc /= np.linalg.norm(xc)
    yc = np.cross(zc, xc)
    height_m = float(abs(np.dot(y_raw, yc)))

    # Per-point canvas coords and out-of-plane residual: the component of
    # (point - tl) along the fitted normal is exactly the offset the flat
    # model misses at that point.
    rel = all_pts - tl
    xs_mm = rel @ xc * 1000.0
    ys_mm = rel @ yc * 1000.0
    resid_mm = rel @ zc * 1000.0
    max_before = float(np.max(np.abs(resid_mm)))

    coeffs = fit_z_correction(xs_mm, ys_mm, resid_mm)
    fitted = _corr_design(xs_mm, ys_mm) @ np.asarray(coeffs, dtype=float)
    max_after = float(np.max(np.abs(resid_mm - fitted)))

    origin = tl + zc * (plane_bias_mm / 1000.0)
    quat = matrix_to_quat(np.column_stack((xc, yc, zc)))
    return origin, quat, width_m, height_m, coeffs, max_before, max_after


class TeachCanvas(Node):
    # All four are required and feed the least-squares plane fit.
    CORNERS = ("top_left", "top_right", "bottom_left", "bottom_right")
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
        # Out-of-plane error left after the Z-correction surface. The pen
        # spring's preload margin to liftoff is only ~plane_bias_mm, so warn
        # early and refuse to save a canvas that cannot physically draw evenly.
        self.declare_parameter("flatness_warn_mm", 0.3)
        self.declare_parameter("flatness_refuse_mm", 0.6)
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
        self.interior_samples = []
        # (monotonic time, pen-tip position) history, twice the record window.
        self.samples = collections.deque(
            maxlen=max(2, int(2 * self.record_window_s * self.SAMPLE_RATE_HZ))
        )
        self._tf_wait_logged = False
        self.create_timer(1.0 / self.SAMPLE_RATE_HZ, self._sample_pen_tip)

        for corner in self.CORNERS:
            self.create_service(
                Trigger,
                f"~/record_{corner}",
                lambda req, res, c=corner: self.record(c, res),
            )
        self.create_service(Trigger, "~/record_sample", self.record_sample)
        self.create_service(Trigger, "~/save", self.save)

        self.get_logger().info(
            "Teach mode: verify joint_trajectory_controller is inactive, "
            "enable pendant freedrive, touch the pen tip to each paper corner "
            "and call ~/record_top_left, ~/record_top_right, "
            "~/record_bottom_left, ~/record_bottom_right (all four required), "
            "optionally ~/record_sample for interior points, then ~/save. "
            "Hands off the arm while recording."
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

    def _capture_point(self):
        """Average the still pen-tip position over the record window.

        Returns (point, n_samples, spread_mm, error_message); point is None
        with a populated error_message when there aren't enough samples or the
        arm moved during the window.
        """
        cutoff = time.monotonic() - self.record_window_s
        window = [p for t, p in self.samples if t >= cutoff]
        if len(window) < self.min_record_samples:
            return None, len(window), 0.0, (
                f"Only {len(window)} pen-tip samples in the last "
                f"{self.record_window_s:.1f} s (need "
                f"{self.min_record_samples}); TF may not be streaming yet — "
                "wait a moment and re-record"
            )
        p, spread_mm = average_still_samples(window, self.stillness_tol_mm)
        if p is None:
            return None, len(window), spread_mm, (
                f"Arm moved {spread_mm:.2f} mm during the last "
                f"{self.record_window_s:.1f} s (tolerance "
                f"{self.stillness_tol_mm:.2f} mm). Release the arm, let it "
                "settle, and re-record"
            )
        return p, len(window), spread_mm, None

    def record(self, corner, res):
        p, n, spread_mm, err = self._capture_point()
        if err is not None:
            res.success = False
            res.message = err
            self.get_logger().warn(err)
            return res
        self.points[corner] = p
        missing = [c for c in self.CORNERS if c not in self.points]
        res.success = True
        res.message = (
            f"{corner} = [{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}] m "
            f"(mean of {n} samples, spread {spread_mm:.2f} mm)"
            + (f"; still missing: {', '.join(missing)}" if missing else
               "; all corners recorded, call ~/save")
        )
        self.get_logger().info(res.message)
        return res

    def record_sample(self, _req, res):
        p, n, spread_mm, err = self._capture_point()
        if err is not None:
            res.success = False
            res.message = err
            self.get_logger().warn(err)
            return res
        self.interior_samples.append(p)
        res.success = True
        res.message = (
            f"interior sample #{len(self.interior_samples)} = "
            f"[{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}] m (mean of {n} samples, "
            f"spread {spread_mm:.2f} mm)"
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
        br = self.points["bottom_right"]
        bias_mm = float(self.get_parameter("plane_bias_mm").value)

        try:
            (origin, (qx, qy, qz, qw), width_m, height_m, corr_coeffs,
             resid_before_mm, resid_after_mm) = compute_canvas_calibration(
                tl, tr, bl, br, self.interior_samples, bias_mm)
        except ValueError as exc:
            res.success = False
            res.message = str(exc)
            return res

        n_samples = len(self.interior_samples)

        # Hard gate: out-of-plane error the correction can't remove will rip
        # one paper edge / gap the other, since the spring cannot absorb it.
        refuse_mm = float(self.get_parameter("flatness_refuse_mm").value)
        if resid_after_mm > refuse_mm:
            res.success = False
            res.message = (
                f"Canvas too non-flat: {resid_after_mm:.2f} mm out-of-plane "
                f"error remains after correction (limit {refuse_mm:.2f} mm; "
                f"{n_samples} interior sample(s), was {resid_before_mm:.2f} mm "
                "before correction). Record more interior samples "
                "(~/record_sample) across the paper, or re-teach the corners "
                "— the pen spring cannot absorb this."
            )
            self.get_logger().error(res.message)
            return res

        warnings = []
        # 3.8 mm is the pen spring's full travel: a negative bias draws in
        # the air, a bias past full travel bottoms the spring out.
        if bias_mm < 0.0 or bias_mm > 3.8:
            warnings.append(
                f"plane_bias_mm {bias_mm:.1f} is outside the pen spring's "
                "0-3.8 mm travel — check the sign/value"
            )
        warn_flat_mm = float(self.get_parameter("flatness_warn_mm").value)
        if resid_after_mm > warn_flat_mm:
            warnings.append(
                f"out-of-plane error {resid_after_mm:.2f} mm after correction "
                f"(warn {warn_flat_mm:.2f} mm); add interior samples across "
                "the paper for a better fit"
            )
        if n_samples < 5:
            warnings.append(
                f"only {n_samples} interior sample(s): the correction surface "
                "is flat/linear. Record ~5-9 with ~/record_sample across the "
                "paper to model reach-dependent contact error"
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
        residual_mm = rectangle_residual_mm(tl, tr, bl, br)
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
                    "canvas_z_correction_coeffs": [
                        round(float(v), 8) for v in corr_coeffs
                    ],
                }
            }
        }
        header = (
            "# Canvas calibration written by teach_canvas.py.\n"
            f"# base frame: {self.base_frame}, ee frame: {self.ee_frame}, "
            f"tool_offset_xyz: {self.tool_offset.tolist()}\n"
            f"# measured paper: {width_m * 1000:.1f} x "
            f"{height_m * 1000:.1f} mm\n"
            f"# out-of-plane error: {resid_before_mm:.2f} mm before "
            f"correction, {resid_after_mm:.2f} mm after "
            f"({n_samples} interior sample(s))\n"
            "# canvas_z_correction_coeffs [a,b,c,d,e,f]: quadratic surface "
            "z(x,y)=a+b*x+c*y+d*x*y+e*x^2+f*y^2 (mm, x/y in canvas mm).\n"
            "# Measured flatness record only — NOT applied by the executor "
            "(see docs/aubo-painting-tracking-remediation-plan.md Section 4).\n"
            f"# plane_bias_mm: {bias_mm} (origin sits this far behind the "
            "raw top_left, along the canvas normal into the wall)\n"
            f"# top_left:     {tl.tolist()}\n"
            f"# top_right:    {tr.tolist()}\n"
            f"# bottom_left:  {bl.tolist()}\n"
            f"# bottom_right: {br.tolist()} "
            f"(rectangle residual {residual_mm:.2f} mm)\n"
            + "".join(
                f"# sample {i + 1}: {np.asarray(s).tolist()}\n"
                for i, s in enumerate(self.interior_samples)
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
            f"{height_m * 1000:.1f} mm, plane bias {bias_mm:.1f} mm, "
            f"out-of-plane {resid_after_mm:.2f} mm after fitted surface; "
            "correction surface recorded as a diagnostic only, not applied "
            "by the executor"
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
