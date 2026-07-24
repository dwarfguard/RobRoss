#!/usr/bin/env python3
"""Offline analysis of painting tracking rosbag2 recordings (Phase 0 of
docs/aubo-painting-tracking-remediation-plan.md).

Reads a rosbag2 SQLite recording WITHOUT replaying it onto the live ROS
graph (nothing initialized, no node, no publishers), reconstructs reference and
actual pen-tip positions through the bag's own robot_description, projects
them onto the taught canvas frame, and reports per-segment tracking metrics
so every implementation change can be compared against the same baseline.

Usage:
  analyze_tracking_bag.py BAG_DIR \
      --canvas-file <teach_canvas output yaml> \
      --calibration-file <hardware_a4.yaml> \
      [--urdf <urdf file, else read from the bag's /robot_description>] \
      [--plane-bias-mm 1.0] [--csv out.csv] [--servoj-csv servoj.csv] \
      [--base-frame base_link] [--ee-frame ee_link]

Conventions (must match painting_executor.cpp):
  - Canvas +z points INTO the paper; a pen tip pushed past the paper plane
    reports positive canvas z.
  - Estimated spring compression = plane_bias_mm + actual_canvas_z_mm
    (the taught origin already sits plane_bias_mm behind the raw touch).

When the bag also carries the Aubo driver's ServoJ diagnostics (Phase 2A:
aubo_servoj_diag /rosout lines), the summary gains a ServoJ timing section
with the effective loop rate, RPC/queue-full/return-code stats, and a
Phase 2B timing-gate check; --servoj-csv writes the per-window series.
"""

import argparse
import csv
import math
import re
import sys
import xml.etree.ElementTree as ET
from collections import namedtuple

import numpy as np
import yaml

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROSOUT_NODE = "painting_executor"
COMMAND_START_RE = re.compile(r"^\[(\d+)/(\d+)\] (\S+) \((.*)\)$")
COMMAND_FAIL_RE = re.compile(r"^Command (\d+) \(.*\) failed, aborting$")
PAINTING_FINISHED_RE = re.compile(r"^Painting finished \((\d+) commands\)$")

# ServoJ timing diagnostics emitted by the Aubo hardware interface's diag node
# (Phase 2A instrumentation). The driver logs one "servoj_config" line at
# activation and one "servoj_stats" key=value line per report window, plus
# throttled "servoj_mismatch"/"servoj_rc"/queue-full warnings.
SERVOJ_NODE = "aubo_servoj_diag"
SERVOJ_CONFIG_PREFIX = "servoj_config"
SERVOJ_STATS_PREFIX = "servoj_stats"

Joint = namedtuple("Joint", "name type xyz rpy axis")
Segment = namedtuple("Segment", "index total type label t_start t_end")


# ---------------------------------------------------------------------------
# Bag reading (read-only, never touches the ROS graph)
# ---------------------------------------------------------------------------

def read_bag(bag_dir, topics=None):
    """Read a rosbag2 directory into {topic: [(t_ns, msg), ...]}.

    Only deserializes the requested topics (all when topics is None).
    """
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_by_topic = {
        t.name: t.type for t in reader.get_all_topics_and_types()
    }
    wanted = set(topics) if topics is not None else set(type_by_topic)
    msg_classes = {}
    out = {t: [] for t in wanted if t in type_by_topic}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in out:
            continue
        cls = msg_classes.get(topic)
        if cls is None:
            cls = get_message(type_by_topic[topic])
            msg_classes[topic] = cls
        out[topic].append((t_ns, deserialize_message(data, cls)))
    return out


# ---------------------------------------------------------------------------
# Forward kinematics from the recorded URDF (the same calibrated chain
# MoveIt uses: it consumes the identical runtime robot_description)
# ---------------------------------------------------------------------------

def _rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def make_transform(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = _rpy_matrix(rpy)
    T[:3, 3] = xyz
    return T


def _axis_angle_matrix(axis, angle):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = a
    return np.array([
        [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
        [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
        [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
    ])


def parse_urdf_chain(urdf_xml, base_frame, ee_frame):
    """Ordered joint chain base_frame -> ee_frame from a URDF string."""
    root = ET.fromstring(urdf_xml)
    by_child = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        jtype = joint.get("type")
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        origin = joint.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            if origin.get("xyz"):
                xyz = [float(v) for v in origin.get("xyz").split()]
            if origin.get("rpy"):
                rpy = [float(v) for v in origin.get("rpy").split()]
        axis_el = joint.find("axis")
        axis = [1.0, 0.0, 0.0]
        if axis_el is not None and axis_el.get("xyz"):
            axis = [float(v) for v in axis_el.get("xyz").split()]
        by_child[child] = (parent, Joint(name, jtype, xyz, rpy, axis))
    chain = []
    link = ee_frame
    while link != base_frame:
        if link not in by_child:
            raise ValueError(
                f"No URDF chain from '{base_frame}' to '{ee_frame}' "
                f"(stuck at link '{link}')")
        link, joint = by_child[link]
        chain.append(joint)
    chain.reverse()
    return chain


def fk(chain, joint_positions):
    """4x4 base->tip transform for {joint_name: position}."""
    T = np.eye(4)
    for j in chain:
        T = T @ make_transform(j.xyz, j.rpy)
        if j.type in ("revolute", "continuous"):
            R = np.eye(4)
            R[:3, :3] = _axis_angle_matrix(j.axis, joint_positions[j.name])
            T = T @ R
        elif j.type == "prismatic":
            P = np.eye(4)
            a = np.asarray(j.axis, dtype=float)
            P[:3, 3] = a / np.linalg.norm(a) * joint_positions[j.name]
            T = T @ P
        elif j.type != "fixed":
            raise ValueError(f"Unsupported joint type '{j.type}' ({j.name})")
    return T


def tip_position(chain, tool_T, joint_positions):
    """Pen-tip position (m) in the base frame."""
    return (fk(chain, joint_positions) @ tool_T)[:3, 3]


# ---------------------------------------------------------------------------
# Canvas frame
# ---------------------------------------------------------------------------

def quat_to_matrix(qx, qy, qz, qw):
    """Rotation matrix from an xyzw quaternion (matches tf2::Matrix3x3)."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
         2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),
         2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
         1 - 2 * (qx * qx + qy * qy)],
    ])


def canvas_frame_from_yaml(canvas_yaml_path):
    """(origin_m, R) from a teach_canvas output YAML.

    R's columns are the canvas axes in the base frame (x right, y down,
    z into the paper), matching CanvasFrame::fromQuaternion.
    """
    with open(canvas_yaml_path) as f:
        data = yaml.safe_load(f)
    params = data["painting_executor"]["ros__parameters"]
    origin = np.asarray(params["canvas_origin_xyz"], dtype=float)
    qx, qy, qz, qw = params["canvas_quat_xyzw"]
    return origin, quat_to_matrix(qx, qy, qz, qw)


def project_to_canvas(origin, R, p):
    """Base-frame point (m) -> canvas (x_mm, y_mm, z_mm); +z into paper."""
    return (R.T @ (np.asarray(p, dtype=float) - origin)) * 1000.0


def tool_transform_from_yaml(calibration_yaml_path):
    """4x4 ee->tip transform from hardware calibration tool_offset_xyz/rpy."""
    with open(calibration_yaml_path) as f:
        data = yaml.safe_load(f)
    params = data["painting_executor"]["ros__parameters"]
    xyz = params.get("tool_offset_xyz", [0.0, 0.0, 0.0])
    rpy = params.get("tool_offset_rpy", [0.0, 0.0, 0.0])
    return make_transform([float(v) for v in xyz], [float(v) for v in rpy])


# ---------------------------------------------------------------------------
# Segmentation from executor /rosout labels
# ---------------------------------------------------------------------------

def segment_rosout(rosout_msgs, end_t_ns=None):
    """Command segments from painting_executor /rosout logs.

    rosout_msgs: [(t_ns, rcl_interfaces/msg/Log)]. A segment runs from its
    start line to the next command start, the "Painting finished" line, the
    "failed, aborting" line, or end_t_ns (default: last log timestamp).
    """
    events = []
    for t_ns, msg in rosout_msgs:
        if msg.name != ROSOUT_NODE:
            continue
        m = COMMAND_START_RE.match(msg.msg)
        if m:
            events.append((t_ns, "start", m))
            continue
        if (PAINTING_FINISHED_RE.match(msg.msg)
                or COMMAND_FAIL_RE.match(msg.msg)):
            events.append((t_ns, "end", None))
    if end_t_ns is None and rosout_msgs:
        end_t_ns = rosout_msgs[-1][0]
    segments = []
    for i, (t_ns, kind, m) in enumerate(events):
        if kind != "start":
            continue
        t_end = end_t_ns
        for t2, _kind2, _m2 in events[i + 1:]:
            t_end = t2
            break
        segments.append(Segment(int(m.group(1)), int(m.group(2)),
                                m.group(3), m.group(4), t_ns, t_end))
    return segments


# ---------------------------------------------------------------------------
# ServoJ timing diagnostics (from the driver's aubo_servoj_diag /rosout lines)
# ---------------------------------------------------------------------------

def _num(s):
    """Parse a diagnostics token value to int/float, leaving non-numbers as-is."""
    try:
        f = float(s)
    except ValueError:
        return s
    return int(f) if f.is_integer() else f


def _parse_kv_line(text):
    """Flatten a diagnostics line into a dict, matching the mixed comma grammar
    emitted by ServoTimingStats::formatReport.

    Within one whitespace token, comma-joined pieces are either a colon
    sub-value of the current group key ('period_ms=min:4.5,mean:5.0' ->
    'period_ms.min'/'period_ms.mean') or an independent 'key=value' pair
    ('late=2,late_run_max=1' -> 'late'/'late_run_max'). Scalars parse
    numerically; the leading tag token (e.g. 'servoj_stats') is skipped.
    """
    out = {}
    for tok in text.split():
        group_key = None
        for piece in tok.split(","):
            if "=" in piece:
                key, val = piece.split("=", 1)
                group_key = key
                if ":" in val:
                    sub, sv = val.split(":", 1)
                    out[f"{key}.{sub}"] = _num(sv)
                else:
                    out[key] = _num(val)
            elif ":" in piece and group_key is not None:
                sub, sv = piece.split(":", 1)
                out[f"{group_key}.{sub}"] = _num(sv)
            # a piece with neither '=' nor ':' (e.g. the leading tag) is ignored
    return out


def parse_servoj_diag(rosout_msgs):
    """Structured ServoJ timing data from aubo_servoj_diag /rosout lines.

    Returns None when the bag has no such lines (e.g. a pre-Phase-2A bag or a
    fake-hardware run). Otherwise: {"config", "reports", "warnings",
    "aggregate"}, where reports is one dict per "servoj_stats" window.
    """
    config = None
    reports = []
    warn = {"mismatch": 0, "rc": 0, "queue_full": 0, "fault_latched": False}
    seen = False
    for t_ns, msg in rosout_msgs:
        if getattr(msg, "name", None) != SERVOJ_NODE:
            continue
        text = msg.msg
        if text.startswith(SERVOJ_CONFIG_PREFIX):
            seen = True
            config = _parse_kv_line(text)
        elif text.startswith(SERVOJ_STATS_PREFIX):
            seen = True
            rep = _parse_kv_line(text)
            rep["t_ns"] = t_ns
            reports.append(rep)
        elif text.startswith("servoj_mismatch"):
            seen = True
            warn["mismatch"] += 1
        elif text.startswith("servoj_rc"):
            seen = True
            warn["rc"] += 1
        elif "queue-full" in text:
            seen = True
            warn["queue_full"] += 1
        elif "fault latched" in text or "latching fault" in text:
            seen = True
            warn["fault_latched"] = True
    if not seen:
        return None
    return {
        "config": config,
        "reports": reports,
        "warnings": warn,
        "aggregate": _aggregate_servoj(config, reports),
    }


def _aggregate_servoj(config, reports):
    """Roll per-window reports into one bag-level summary + Phase 2B gate."""
    if not reports:
        return None
    total_cycles = sum(r.get("cycles", 0) for r in reports)

    def wmean(key):  # cycle-weighted mean across windows
        if not total_cycles:
            return 0.0
        return sum(r.get(key, 0.0) * r.get("cycles", 0)
                   for r in reports) / total_cycles

    def cmax(key):
        return max((r.get(key, 0.0) for r in reports), default=0.0)

    def csum(key):
        return sum(r.get(key, 0) for r in reports)

    period_mean_ms = wmean("period_ms.mean")
    agg = {
        "n_reports": len(reports),
        "total_cycles": total_cycles,
        "period_mean_ms": period_mean_ms,
        "period_min_ms": min((r.get("period_ms.min", 0.0) for r in reports),
                             default=0.0),
        "period_max_ms": cmax("period_ms.max"),
        # Percentiles can't be exactly merged across windows; report the worst
        # window's value as a conservative bound.
        "period_p95_ms_worst": cmax("period_ms.p95"),
        "period_p99_ms_worst": cmax("period_ms.p99"),
        "rpc_mean_ms": wmean("rpc_ms.mean"),
        "rpc_max_ms": cmax("rpc_ms.max"),
        "total_mean_ms": wmean("total_ms.mean"),
        "total_max_ms": cmax("total_ms.max"),
        "late_cycles": csum("late"),
        "late_run_max": cmax("late_run_max"),
        "qf_events": csum("qf_events"),
        "qf_retries": csum("qf_retries"),
        "qf_blocked_ms": sum(r.get("qf_blocked_ms", 0.0) for r in reports),
        "rc_ok": csum("rc.ok"),
        "rc_busy": csum("rc.busy"),
        "rc_bad": csum("rc.bad"),
        "rc_inval": csum("rc.inval"),
        "rc_ign": csum("rc.ign"),
        "rc_other": csum("rc.other"),
        "exc": csum("exc"),
    }
    agg["non_ok_rc"] = (agg["rc_busy"] + agg["rc_bad"] + agg["rc_inval"]
                        + agg["rc_ign"] + agg["rc_other"])
    agg["effective_rate_hz"] = (1000.0 / period_mean_ms
                                if period_mean_ms > 0 else 0.0)
    t_s = config.get("t") if config else None
    if t_s:
        agg["configured_rate_hz"] = 1.0 / t_s
        agg["rate_pct_of_configured"] = 100.0 * agg["effective_rate_hz"] * t_s
    else:
        agg["configured_rate_hz"] = None
        agg["rate_pct_of_configured"] = None
    return agg


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def _point_positions(state_msg, primary, legacy):
    pt = getattr(state_msg, primary, None)
    if pt is not None and len(pt.positions) > 0:
        return pt.positions
    return getattr(state_msg, legacy).positions


def classify_direction(xy_mm, min_travel_mm=1.0, dominance=3.0):
    """'+X'/'-X'/'+Y'/'-Y'/'mixed'/'static' from net canvas displacement."""
    xy = np.asarray(xy_mm, dtype=float)
    if len(xy) < 2:
        return "static"
    d = xy[-1] - xy[0]
    ax, ay = abs(d[0]), abs(d[1])
    if max(ax, ay) < min_travel_mm:
        return "static"
    if ax >= dominance * ay:
        return "+X" if d[0] > 0 else "-X"
    if ay >= dominance * ax:
        return "+Y" if d[1] > 0 else "-Y"
    return "mixed"


def _instantaneous_direction(vx, vy, min_speed_mm_s, dominance=3.0):
    """Direction label from a velocity vector (classify_direction's dominance
    rule applied per-sample rather than to net displacement)."""
    ax, ay = abs(vx), abs(vy)
    if max(ax, ay) < min_speed_mm_s:
        return "static"
    if ax >= dominance * ay:
        return "+X" if vx > 0 else "-X"
    if ay >= dominance * ax:
        return "+Y" if vy > 0 else "-Y"
    return "mixed"


def sample_velocities_mm_s(xy_mm, t_s):
    """Central-difference canvas velocity (mm/s) per sample; handles the
    controller-state timestamp jitter via np.gradient over the actual times."""
    xy = np.asarray(xy_mm, dtype=float)
    t = np.asarray(t_s, dtype=float)
    if len(xy) < 2:
        return np.zeros((len(xy), 2))
    return np.gradient(xy, t, axis=0)


def direction_resolved_normal_err(xy_mm, normal_err_mm, t_s,
                                  min_speed_mm_s=2.0, dominance=3.0):
    """Normal-error stats grouped by INSTANTANEOUS canvas direction.

    Splits a reversal/curve command by the direction of motion at each sample
    (returns the +X/-X/+Y/-Y buckets that have samples) instead of reducing the
    whole command to its net-displacement direction. Diagonal samples (neither
    canvas axis dominant) are retained in their own "mixed" bucket rather than
    discarded: a curved command's largest canvas-normal error often falls on
    the diagonal portion, so dropping it would hide the worst tracking error.
    Only genuinely stationary ("static") samples are excluded.
    """
    v = sample_velocities_mm_s(xy_mm, t_s)
    ne = np.asarray(normal_err_mm, dtype=float)
    buckets = {}
    for i in range(len(ne)):
        d = _instantaneous_direction(v[i, 0], v[i, 1], min_speed_mm_s, dominance)
        if d == "static":
            continue
        buckets.setdefault(d, []).append(ne[i])
    out = {}
    for d, vals in buckets.items():
        a = np.asarray(vals, dtype=float)
        out[d] = {"n": int(a.size), "mean_mm": float(a.mean()),
                  "min_mm": float(a.min()), "max_mm": float(a.max())}
    return out


def estimate_phase_delay_s(t_s, ref, act, max_lag_s=0.3, min_std=1e-4):
    """Delay (s) by which `act` lags `ref`, via a best-correlation integer-lag
    search on a uniform resample. Returns None when the signal is too short or
    too quiet to yield a meaningful lag (e.g. a joint that barely moves)."""
    t = np.asarray(t_s, dtype=float)
    ref = np.asarray(ref, dtype=float)
    act = np.asarray(act, dtype=float)
    if len(t) < 8:
        return None
    dt = float(np.median(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0:
        return None
    grid = np.arange(t[0], t[-1], dt)
    if len(grid) < 8:
        return None
    r = np.interp(grid, t, ref)
    a = np.interp(grid, t, act)
    if r.std() < min_std or a.std() < min_std:
        return None
    # A near-straight-line reference (a monotonic stroke) correlates equally
    # well at every lag, so its command-to-feedback delay is undefined. Only
    # oscillatory/curved references (e.g. the sine fixture) yield a meaningful
    # lag; reject signals whose nonlinear component is negligible.
    resid = r - np.polyval(np.polyfit(grid, r, 1), grid)
    if resid.std() < 0.05 * r.std():
        return None
    max_lag = int(min(max_lag_s / dt, len(grid) - 4))
    if max_lag < 1:
        return None
    best_lag, best_corr = 0, -2.0
    for lag in range(max_lag + 1):
        aa = a[lag:]
        rr = r[:len(a) - lag]
        aa = aa - aa.mean()
        rr = rr - rr.mean()
        denom = math.sqrt(float((aa * aa).sum()) * float((rr * rr).sum()))
        if denom <= 0.0:
            continue
        corr = float((aa * rr).sum()) / denom
        if corr > best_corr:
            best_corr, best_lag = corr, lag
    return best_lag * dt


def per_joint_phase_delay_ms(t_s, ref_by_name, act_by_name, max_lag_s=0.3):
    """Per-joint command-to-feedback delay (ms) for joints that actually move
    (quiet joints are omitted rather than reported as a spurious 0)."""
    out = {}
    for n in ref_by_name:
        d = estimate_phase_delay_s(t_s, ref_by_name[n], act_by_name.get(n, []),
                                   max_lag_s=max_lag_s)
        if d is not None:
            out[n] = d * 1000.0
    return out


def _reversal_indices(pos, min_step):
    """Indices where the dominant-axis motion reverses, ignoring sub-min_step
    jitter so controller-state noise does not fabricate cycles."""
    revs = []
    last_sign = 0.0
    for i in range(1, len(pos)):
        d = pos[i] - pos[i - 1]
        if abs(d) < min_step:
            continue
        s = 1.0 if d > 0 else -1.0
        if last_sign != 0.0 and s != last_sign:
            revs.append(i)
        last_sign = s
    return revs


def normal_pp_per_cycle(tangential_xy_mm, normal_err_mm, min_step_mm=0.05):
    """Per-cycle peak-to-peak of the canvas-normal error for an oscillating
    fixture (e.g. the sine path). A cycle spans two reversals of the dominant
    tangential axis; falls back to the whole segment under one full cycle."""
    xy = np.asarray(tangential_xy_mm, dtype=float)
    ne = np.asarray(normal_err_mm, dtype=float)
    n = len(ne)
    whole = float(ne.max() - ne.min()) if n else 0.0
    if n < 3:
        return {"n_cycles": 0, "pp_mean_mm": whole, "pp_max_mm": whole}
    # Pick the OSCILLATING tangential axis (the one that actually reverses),
    # not the axis with the greatest total travel. The real sine fixture
    # advances ~90 mm monotonically in X while oscillating ~48 mm in Y, so
    # argmax(range) would pick monotonic X, find no reversals, and report
    # zero cycles. Compare each axis's reversal count and fall back to the
    # widest axis only on a tie (e.g. neither axis reverses).
    revs_by_axis = [_reversal_indices(xy[:, k], min_step_mm) for k in (0, 1)]
    if len(revs_by_axis[0]) == len(revs_by_axis[1]):
        axis = int(np.argmax(xy.max(axis=0) - xy.min(axis=0)))
    else:
        axis = 0 if len(revs_by_axis[0]) > len(revs_by_axis[1]) else 1
    bounds = [0] + revs_by_axis[axis] + [n - 1]
    windows = [(bounds[k], bounds[k + 2])
               for k in range(0, len(bounds) - 2, 2)]
    pps = [float(ne[a:b + 1].max() - ne[a:b + 1].min())
           for a, b in windows if b > a]
    if not pps:
        return {"n_cycles": 0, "pp_mean_mm": whole, "pp_max_mm": whole}
    return {"n_cycles": len(pps), "pp_mean_mm": float(np.mean(pps)),
            "pp_max_mm": float(np.max(pps))}


def compute_segment_metrics(segment, ctrl_msgs, chain, tool_T, origin, R,
                            plane_bias_mm):
    """Tracking metrics for one command segment.

    ctrl_msgs: [(t_ns, JointTrajectoryControllerState)]. Returns None when
    no controller-state samples fall inside the segment, else a dict with
    per-joint and canvas-space error statistics plus per-sample rows.
    """
    rows = []
    joint_err_by_name = {}
    ref_by_name = {}
    act_by_name = {}
    for t_ns, msg in ctrl_msgs:
        # Half-open window: a sample on the boundary belongs to the segment
        # that starts there, not the one that just ended.
        if not (segment.t_start <= t_ns < segment.t_end):
            continue
        names = list(msg.joint_names)
        ref = _point_positions(msg, "reference", "desired")
        act = _point_positions(msg, "feedback", "actual")
        ref_map = dict(zip(names, ref))
        act_map = dict(zip(names, act))
        for n in names:
            joint_err_by_name.setdefault(n, []).append(
                act_map[n] - ref_map[n])
            ref_by_name.setdefault(n, []).append(ref_map[n])
            act_by_name.setdefault(n, []).append(act_map[n])
        ref_c = project_to_canvas(origin, R, tip_position(chain, tool_T,
                                                          ref_map))
        act_c = project_to_canvas(origin, R, tip_position(chain, tool_T,
                                                          act_map))
        rows.append((t_ns, ref_c, act_c))
    if not rows:
        return None

    ref_arr = np.array([r[1] for r in rows])
    act_arr = np.array([r[2] for r in rows])
    err = act_arr - ref_arr
    normal_err = err[:, 2]
    tangential_err = np.linalg.norm(err[:, :2], axis=1)
    t_arr = np.array([r[0] for r in rows], dtype=float) / 1e9
    duration = t_arr[-1] - t_arr[0] if len(t_arr) > 1 else 0.0
    path_len = float(np.sum(np.linalg.norm(np.diff(ref_arr[:, :2], axis=0),
                                           axis=1))) if len(rows) > 1 else 0.0
    compression = plane_bias_mm + act_arr[:, 2]
    return {
        "segment": segment,
        "n_samples": len(rows),
        "duration_s": duration,
        "direction": classify_direction(ref_arr[:, :2]),
        "speed_mm_s": path_len / duration if duration > 0 else 0.0,
        "normal_err_mean_mm": float(np.mean(normal_err)),
        "normal_err_min_mm": float(np.min(normal_err)),
        "normal_err_max_mm": float(np.max(normal_err)),
        "normal_err_rms_mm": float(np.sqrt(np.mean(np.square(normal_err)))),
        "normal_err_pp_mm": float(np.max(normal_err) - np.min(normal_err)),
        "tangential_err_max_mm": float(np.max(tangential_err)),
        "compression_mean_mm": float(np.mean(compression)),
        "compression_min_mm": float(np.min(compression)),
        "compression_max_mm": float(np.max(compression)),
        "joint_err_max_deg": {
            n: math.degrees(max(abs(v) for v in e))
            for n, e in joint_err_by_name.items()
        },
        "joint_err_mean_deg": {
            n: math.degrees(float(np.mean(np.abs(e))))
            for n, e in joint_err_by_name.items()
        },
        "joint_err_rms_deg": {
            n: math.degrees(float(np.sqrt(np.mean(np.square(e)))))
            for n, e in joint_err_by_name.items()
        },
        # Command-to-feedback delay per moving joint (§2.2 phase lag).
        "joint_delay_ms": per_joint_phase_delay_ms(t_arr, ref_by_name,
                                                   act_by_name),
        # Normal error resolved by instantaneous direction (reversals/curves).
        "direction_resolved": direction_resolved_normal_err(
            ref_arr[:, :2], normal_err, t_arr),
        # Per-cycle canvas-normal peak-to-peak (sine/oscillation, §2.6).
        "normal_pp_per_cycle": normal_pp_per_cycle(ref_arr[:, :2], normal_err),
        "rows": rows,
    }


def rate_stats(stamps_ns):
    """Publication rate/jitter stats for a list of bag timestamps (ns)."""
    if len(stamps_ns) < 2:
        return None
    dt = np.diff(np.asarray(stamps_ns, dtype=float)) / 1e9
    dt = dt[dt > 0]
    if len(dt) == 0:
        return None
    return {
        "n": len(stamps_ns),
        "rate_mean_hz": float(1.0 / np.mean(dt)),
        "rate_median_hz": float(1.0 / np.median(dt)),
        "interval_min_ms": float(np.min(dt) * 1000.0),
        "interval_mean_ms": float(np.mean(dt) * 1000.0),
        "interval_max_ms": float(np.max(dt) * 1000.0),
        "interval_p95_ms": float(np.percentile(dt, 95) * 1000.0),
    }


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def write_csv(path, metrics_list):
    """Per-sample CSV: one row per controller-state sample per segment."""
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "t_ns", "command_index", "command_type", "label",
            "ref_canvas_x_mm", "ref_canvas_y_mm", "ref_canvas_z_mm",
            "act_canvas_x_mm", "act_canvas_y_mm", "act_canvas_z_mm",
            "normal_err_mm", "tangential_err_mm",
        ])
        for m in metrics_list:
            seg = m["segment"]
            for t_ns, ref_c, act_c in m["rows"]:
                err = act_c - ref_c
                w.writerow([
                    t_ns, seg.index, seg.type, seg.label,
                    f"{ref_c[0]:.4f}", f"{ref_c[1]:.4f}", f"{ref_c[2]:.4f}",
                    f"{act_c[0]:.4f}", f"{act_c[1]:.4f}", f"{act_c[2]:.4f}",
                    f"{err[2]:.4f}",
                    f"{math.hypot(err[0], err[1]):.4f}",
                ])


def write_servoj_csv(path, servoj):
    """One row per servoj_stats window: the per-window timing series (for
    plotting/comparing A/B ServoJ timing trials)."""
    cols = [
        "t_ns", "cycles", "period_ms.min", "period_ms.mean", "period_ms.max",
        "period_ms.p95", "period_ms.p99", "rpc_ms.mean", "rpc_ms.max",
        "total_ms.mean", "total_ms.max", "late", "late_run_max",
        "qf_events", "qf_retries", "qf_blocked_ms",
        "rc.ok", "rc.busy", "rc.bad", "rc.inval", "rc.ign", "rc.other", "exc",
    ]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for r in servoj["reports"]:
            w.writerow([r.get(c, "") for c in cols])


def render_summary(metrics_list, rates, servoj=None):
    lines = []
    lines.append("# Tracking bag analysis")
    lines.append("")
    for topic, st in rates.items():
        if st is None:
            lines.append(f"{topic}: <2 samples")
            continue
        lines.append(
            f"{topic}: {st['n']} msgs, {st['rate_mean_hz']:.1f} Hz mean "
            f"({st['rate_median_hz']:.1f} Hz median), interval "
            f"{st['interval_min_ms']:.1f}/{st['interval_mean_ms']:.1f}/"
            f"{st['interval_max_ms']:.1f} ms min/mean/max, "
            f"p95 {st['interval_p95_ms']:.1f} ms")
    lines.append("")
    lines.append(
        "| # | type | label | dir | n | speed mm/s | normal err mean/min/max"
        " mm | tang max mm | compression mean/min/max mm |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for m in metrics_list:
        seg = m["segment"]
        lines.append(
            f"| {seg.index}/{seg.total} | {seg.type} | {seg.label} "
            f"| {m['direction']} | {m['n_samples']} "
            f"| {m['speed_mm_s']:.1f} "
            f"| {m['normal_err_mean_mm']:+.3f}/{m['normal_err_min_mm']:+.3f}"
            f"/{m['normal_err_max_mm']:+.3f} "
            f"| {m['tangential_err_max_mm']:.3f} "
            f"| {m['compression_mean_mm']:.2f}/{m['compression_min_mm']:.2f}"
            f"/{m['compression_max_mm']:.2f} |")
    lines.append("")
    lines.append("Per-joint max |error| (deg):")
    for m in metrics_list:
        seg = m["segment"]
        worst = ", ".join(
            f"{n}: {v:.2f}" for n, v in sorted(
                m["joint_err_max_deg"].items(), key=lambda kv: -kv[1])[:3])
        lines.append(f"  [{seg.index}] {seg.label}: {worst}")

    if metrics_list:
        lines.append("")
        lines.extend(_render_tracking(metrics_list))
    if servoj and servoj.get("reports"):
        lines.append("")
        lines.extend(_render_servoj(servoj))
    return "\n".join(lines)


def _tracking_gate(metrics_list):
    """Global command-to-feedback delay + canvas-normal summary (Phase 2B
    tracking portion of the §7 gate)."""
    delays = []
    for m in metrics_list:
        delays.extend(m.get("joint_delay_ms", {}).values())
    worst_normal_abs = max(
        (max(abs(m["normal_err_min_mm"]), abs(m["normal_err_max_mm"]))
         for m in metrics_list), default=0.0)
    worst_pp = max((m["normal_pp_per_cycle"]["pp_max_mm"] for m in metrics_list),
                   default=0.0)
    return {
        "delay_median_ms": float(np.median(delays)) if delays else None,
        "delay_p95_ms": float(np.percentile(delays, 95)) if delays else None,
        "worst_normal_abs_mm": worst_normal_abs,
        "worst_normal_pp_mm": worst_pp,
    }


def _render_tracking(metrics_list):
    lines = ["## Phase delay & normal oscillation"]
    lines.append("Instantaneous-direction normal error mean/min/max mm (n):")
    for m in metrics_list:
        seg = m["segment"]
        dr = m["direction_resolved"]
        if not dr:
            lines.append(f"  [{seg.index}] {seg.label}: (no directed motion)")
            continue
        parts = "; ".join(
            f"{d} {s['mean_mm']:+.3f}/{s['min_mm']:+.3f}/{s['max_mm']:+.3f} "
            f"(n={s['n']})" for d, s in sorted(dr.items()))
        lines.append(f"  [{seg.index}] {seg.label}: {parts}")
    lines.append("")
    lines.append("Per-segment delay and canvas-normal oscillation:")
    for m in metrics_list:
        seg = m["segment"]
        delays = m["joint_delay_ms"]
        cyc = m["normal_pp_per_cycle"]
        if delays:
            dmed = float(np.median(list(delays.values())))
            worst = ", ".join(
                f"{n}:{v:.0f}" for n, v in sorted(
                    delays.items(), key=lambda kv: -kv[1])[:2])
            dtxt = f"delay median {dmed:.0f} ms ({worst})"
        else:
            dtxt = "delay n/a"
        lines.append(
            f"  [{seg.index}] {seg.label}: {dtxt}; normal pp seg "
            f"{m['normal_err_pp_mm']:.2f} mm, per-cycle mean/max "
            f"{cyc['pp_mean_mm']:.2f}/{cyc['pp_max_mm']:.2f} mm "
            f"({cyc['n_cycles']} cyc), rms {m['normal_err_rms_mm']:.3f} mm")
    g = _tracking_gate(metrics_list)
    # Command-to-feedback delay is a MANDATORY Phase 2B criterion. Linear-only
    # bags produce no delay estimate (a monotonic ramp correlates at every lag),
    # so absence of a delay measurement must read INCOMPLETE, never PASS -
    # absence of evidence is not evidence the gate was met.
    delay_available = g["delay_median_ms"] is not None
    checks = []
    if delay_available:
        checks.append(("delay median <30 ms", g["delay_median_ms"] < 30.0))
        checks.append(("delay p95 <50 ms", g["delay_p95_ms"] < 50.0))
    checks.append(("|normal| <=0.25 mm", g["worst_normal_abs_mm"] <= 0.25))
    measured_pass = all(v for _, v in checks)
    if not measured_pass:
        status = "FAIL"
    elif not delay_available:
        status = "INCOMPLETE"
    else:
        status = "PASS"
    detail_parts = [f"{name}: {'ok' if v else 'FAIL'}" for name, v in checks]
    if not delay_available:
        detail_parts.insert(
            0, "delay: MISSING (mandatory; need an oscillatory/curved path)")
    detail = ", ".join(detail_parts)
    lines.append(f"Phase 2B tracking gate: {status} ({detail})")
    dmed = ("n/a" if g["delay_median_ms"] is None
            else f"{g['delay_median_ms']:.0f}/{g['delay_p95_ms']:.0f} ms")
    lines.append(
        f"  delay median/p95 {dmed}; worst |normal| "
        f"{g['worst_normal_abs_mm']:.2f} mm; worst per-cycle pp "
        f"{g['worst_normal_pp_mm']:.2f} mm (the per-cycle pp is the objective "
        "proxy for the operator's 'visible wrist oscillation' check).")
    return lines


def _render_servoj(servoj):
    agg = servoj["aggregate"]
    cfg = servoj["config"] or {}
    warn = servoj["warnings"]
    lines = ["## ServoJ timing (aubo_servoj_diag)"]
    if cfg:
        lines.append(
            f"config: t={cfg.get('t')} s, gain={cfg.get('gain')}, "
            f"window={cfg.get('window')} cycles")
    rate_txt = f"{agg['effective_rate_hz']:.1f} Hz effective loop"
    if agg["configured_rate_hz"]:
        rate_txt += (f" ({agg['rate_pct_of_configured']:.1f}% of "
                     f"{agg['configured_rate_hz']:.0f} Hz configured)")
    lines.append(
        f"{agg['n_reports']} reports over {agg['total_cycles']} cycles; "
        + rate_txt)
    lines.append(
        f"period ms: mean {agg['period_mean_ms']:.2f}, "
        f"min {agg['period_min_ms']:.2f}, max {agg['period_max_ms']:.2f}, "
        f"p95 {agg['period_p95_ms_worst']:.2f} / p99 "
        f"{agg['period_p99_ms_worst']:.2f} (worst window)")
    lines.append(
        f"servoJoint RPC ms: mean {agg['rpc_mean_ms']:.2f}, "
        f"max {agg['rpc_max_ms']:.2f}; whole Servoj ms: mean "
        f"{agg['total_mean_ms']:.2f}, max {agg['total_max_ms']:.2f}")
    lines.append(
        f"late cycles: {agg['late_cycles']} (worst run {agg['late_run_max']}); "
        f"queue-full: {agg['qf_events']} events, {agg['qf_retries']} retries, "
        f"{agg['qf_blocked_ms']:.1f} ms blocked")
    lines.append(
        f"return codes: ok {agg['rc_ok']}, busy {agg['rc_busy']}, "
        f"bad {agg['rc_bad']}, inval {agg['rc_inval']}, ign {agg['rc_ign']}, "
        f"other {agg['rc_other']}; exceptions {agg['exc']}")
    lines.append(
        f"log warnings: mismatch {warn['mismatch']}, rc {warn['rc']}, "
        f"queue-full {warn['queue_full']}; fault latched: "
        f"{'YES' if warn['fault_latched'] else 'no'}")
    # The configured rate is a MANDATORY input: without a servoj_config line the
    # bag cannot prove it ran at the intended rate, so the gate reads INCOMPLETE
    # (never a silent PASS with the rate check dropped). Queue-full and non-OK
    # return-code WARNINGS are folded in alongside the per-window stats counts so
    # events in the trailing, never-reported window still fail the gate.
    rate_known = agg["rate_pct_of_configured"] is not None
    checks = []
    if rate_known:
        checks.append(("rate >= 95%", agg["rate_pct_of_configured"] >= 95.0))
    checks.append(("no queue-full",
                   agg["qf_events"] == 0 and warn["queue_full"] == 0))
    checks.append(("no non-OK rc/exc",
                   agg["non_ok_rc"] == 0 and agg["exc"] == 0
                   and warn["rc"] == 0))
    checks.append(("no timing fault", not warn["fault_latched"]))
    measured_pass = all(v for _, v in checks)
    if not measured_pass:
        status = "FAIL"
    elif not rate_known:
        status = "INCOMPLETE"
    else:
        status = "PASS"
    detail_parts = [f"{name}: {'ok' if v else 'FAIL'}" for name, v in checks]
    if not rate_known:
        detail_parts.insert(0, "rate: UNKNOWN (no servoj_config in bag)")
    detail = ", ".join(detail_parts)
    lines.append(f"Phase 2B timing gate: {status} ({detail})")
    lines.append(
        "  (joint-delay gate <30 ms median / <50 ms p95 is assessed separately "
        "from controller_state cross-correlation.)")
    return lines


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def analyze(bag_dir, canvas_file, calibration_file, urdf=None,
            plane_bias_mm=1.0, base_frame="base_link", ee_frame="ee_link"):
    """Full analysis; returns (metrics_list, rates). Read-only."""
    topics = [
        "/robot_description", "/rosout",
        "/joint_trajectory_controller/controller_state", "/joint_states",
    ]
    bag = read_bag(bag_dir, topics)

    if urdf is None:
        desc = bag.get("/robot_description", [])
        if not desc:
            raise ValueError(
                "Bag has no /robot_description; pass --urdf explicitly")
        urdf = desc[0][1].data
    chain = parse_urdf_chain(urdf, base_frame, ee_frame)
    tool_T = tool_transform_from_yaml(calibration_file)
    origin, R = canvas_frame_from_yaml(canvas_file)

    rosout = bag.get("/rosout", [])
    segments = segment_rosout(rosout)
    ctrl = bag.get("/joint_trajectory_controller/controller_state", [])
    metrics_list = []
    for seg in segments:
        m = compute_segment_metrics(seg, ctrl, chain, tool_T, origin, R,
                                    plane_bias_mm)
        if m is not None:
            metrics_list.append(m)
    rates = {
        "/joint_trajectory_controller/controller_state":
            rate_stats([t for t, _ in ctrl]),
        "/joint_states":
            rate_stats([t for t, _ in bag.get("/joint_states", [])]),
    }
    servoj = parse_servoj_diag(rosout)
    return metrics_list, rates, servoj


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bag_dir", help="rosbag2 directory (SQLite)")
    ap.add_argument("--canvas-file", required=True,
                    help="teach_canvas output YAML (taught canvas pose)")
    ap.add_argument("--calibration-file", required=True,
                    help="hardware calibration YAML (tool_offset_xyz/rpy)")
    ap.add_argument("--urdf", help="URDF file; default: bag's "
                                   "/robot_description")
    ap.add_argument("--plane-bias-mm", type=float, default=1.0,
                    help="taught plane bias for compression estimate")
    ap.add_argument("--csv", help="write per-sample CSV here")
    ap.add_argument("--servoj-csv",
                    help="write per-window ServoJ timing CSV here")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--ee-frame", default="ee_link")
    args = ap.parse_args(argv)

    urdf = None
    if args.urdf:
        with open(args.urdf) as f:
            urdf = f.read()
    metrics_list, rates, servoj = analyze(
        args.bag_dir, args.canvas_file, args.calibration_file, urdf=urdf,
        plane_bias_mm=args.plane_bias_mm, base_frame=args.base_frame,
        ee_frame=args.ee_frame)
    if args.csv:
        write_csv(args.csv, metrics_list)
    if args.servoj_csv and servoj and servoj.get("reports"):
        write_servoj_csv(args.servoj_csv, servoj)
    print(render_summary(metrics_list, rates, servoj))
    return 0 if (metrics_list or (servoj and servoj.get("reports"))) else 1


if __name__ == "__main__":
    sys.exit(main())
