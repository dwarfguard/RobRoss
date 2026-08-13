#!/usr/bin/env python3
"""Offline remediation and debugging analysis of RobRoss rosbag2 recordings.

Reads a rosbag2 SQLite recording WITHOUT replaying it onto the live ROS
graph (nothing initialized, no node, no publishers). The default debug report
inventories evidence and detects driver faults, RTDE/source-health failures,
non-finite or frozen feedback, unsafe post-fault command continuation,
post-execution fallback, ServoJ failures, and wall-clock discontinuities.

Optional canvas and calibration files add the historical FK/tracking report.
Strict remediation gates are opt-in because later remediation phases may not yet
be implemented; unavailable capabilities are reported explicitly.

Usage:
  analyze_tracking_bag.py BAG_DIR [--gate stationary-soak] \
      [--servoj-calls-csv full_rate.csv] [--json report.json] \
      [--events-csv events.csv]

  analyze_tracking_bag.py BAG_DIR --canvas-file canvas.yaml \
      --calibration-file hardware.yaml [--csv tracking.csv]

Conventions (must match painting_executor.cpp):
  - Canvas +z points INTO the paper; a pen tip pushed past the paper plane
    reports positive canvas z.
  - Estimated spring compression = plane_bias_mm + actual_canvas_z_mm
    (the taught origin already sits plane_bias_mm behind the raw touch).

When the bag also carries the Aubo driver's ServoJ diagnostics (
aubo_servoj_diag /rosout lines), the summary gains a ServoJ timing section
with the effective loop rate, RPC/queue-full/return-code stats, and a
ServoJ timing screen; --servoj-csv writes the per-window series.
"""

import argparse
import csv
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from collections import namedtuple
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path

import numpy as np
import yaml

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROSOUT_NODE = "painting_executor"
COMMAND_START_RE = re.compile(r"^\[(\d+)/(\d+)\] (\S+) \((.*)\)$")
COMMAND_FAIL_RE = re.compile(r"^Command (\d+) \(.*\) failed, aborting$")
PAINTING_FINISHED_RE = re.compile(r"^Painting finished \((\d+) commands\)$")
DRIVER_HEALTH_TOPIC = "/aubo_driver/health"
DEFAULT_EXPECTED_RATE_HZ = 125.0
DEFAULT_EXPECTED_SERVOJ_T_S = 0.008

# ServoJ timing diagnostics emitted by the Aubo hardware interface's diag node.
# The driver logs one "servoj_config" line at
# activation and one "servoj_stats" key=value line per report window, plus
# throttled "servoj_mismatch"/"servoj_rc"/queue-full warnings.
SERVOJ_NODE = "aubo_servoj_diag"
SERVOJ_CONFIG_PREFIX = "servoj_config"
SERVOJ_STATS_PREFIX = "servoj_stats"

Joint = namedtuple("Joint", "name type xyz rpy axis")
Segment = namedtuple("Segment", "index total type label t_start t_end")


class CheckState(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    UNKNOWN = "UNKNOWN"
    NOT_APPLICABLE = "NOT_APPLICABLE"


@dataclass
class EvidenceCheck:
    key: str
    title: str
    state: CheckState
    summary: str
    confidence: str = "direct"
    t_ns: int = None
    evidence: list = field(default_factory=list)


@dataclass
class IncidentEvent:
    t_ns: int
    category: str
    severity: str
    summary: str
    source: str


@dataclass
class RemediationReport:
    bag_dir: str
    mode: str
    gate: str
    bag_start_ns: int
    bag_end_ns: int
    topic_counts: dict
    checks: list
    events: list
    overall: str
    exit_code: int


GATE_REQUIREMENTS = {
    "stationary-soak": {
        "driver_health_evidence", "driver_fault_free", "feedback_finite",
        "feedback_not_frozen", "servoj_telemetry", "clock_stable",
    },
    "hover": {
        "driver_health_evidence", "driver_fault_free", "feedback_finite",
        "feedback_not_frozen", "no_command_after_fault",
        "no_post_execution_fallback", "moveit_controller_consistency",
        "servoj_telemetry", "clock_stable",
    },
    "deliberate-fault": {
        "driver_health_evidence", "driver_fault_detected",
        "no_command_after_fault", "no_post_execution_fallback",
        "bounded_recovery", "structured_motion_outcomes",
    },
    "contact": {
        "driver_health_evidence", "driver_fault_free", "feedback_finite",
        "feedback_not_frozen", "no_command_after_fault",
        "no_post_execution_fallback", "servoj_telemetry", "clock_stable",
        "bounded_recovery", "moveit_controller_consistency",
    },
    "static-fault": {
        "driver_health_evidence", "driver_fault_detected",
        "no_command_after_fault", "no_post_execution_fallback",
        "structured_motion_outcomes", "controller_hardening",
        "runtime_provenance",
    },
}


def _check(key, title, state, summary, **kwargs):
    return EvidenceCheck(key, title, CheckState(state), summary, **kwargs)


def _message_stamp_ns(msg, fallback_ns):
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return fallback_ns
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else fallback_ns


def _find_topic(topic_types, canonical):
    if canonical in topic_types:
        return canonical
    matches = [name for name in topic_types
               if name == canonical or name.endswith(canonical)]
    return matches[0] if len(matches) == 1 else None


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


def read_bag_evidence(bag_dir, canonical_topics):
    """Read requested evidence and retain the bag's topic inventory."""
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    selected = {}
    for canonical in canonical_topics:
        actual = _find_topic(topic_types, canonical)
        if actual is not None:
            selected[actual] = canonical
    messages = {canonical: [] for canonical in selected.values()}
    counts = {name: 0 for name in topic_types}
    msg_classes = {}
    start_ns = None
    end_ns = None
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        counts[topic] += 1
        start_ns = t_ns if start_ns is None else min(start_ns, t_ns)
        end_ns = t_ns if end_ns is None else max(end_ns, t_ns)
        canonical = selected.get(topic)
        if canonical is None:
            continue
        cls = msg_classes.get(topic)
        if cls is None:
            cls = get_message(topic_types[topic])
            msg_classes[topic] = cls
        messages[canonical].append(
            (t_ns, deserialize_message(data, cls)))
    return {
        "messages": messages,
        "topic_types": topic_types,
        "topic_counts": counts,
        "topic_aliases": {canonical: actual
                          for actual, canonical in selected.items()},
        "start_ns": start_ns,
        "end_ns": end_ns,
    }


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
    """Roll per-window reports into one bag-level timing summary."""
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
# Remediation evidence and incident diagnostics
# ---------------------------------------------------------------------------

def _fault_events(health_msgs, rosout_msgs):
    events = []
    faulted_activations = set()
    for t_ns, msg in health_msgs:
        activation = int(msg.activation_sequence)
        if ((int(msg.state) == 2 or int(msg.fault_code) != 0) and
                activation not in faulted_activations):
            faulted_activations.add(activation)
            events.append(IncidentEvent(
                t_ns, "driver_fault", "error",
                f"Driver fault {int(msg.fault_code)}: {msg.fault_reason}",
                DRIVER_HEALTH_TOPIC))
    for t_ns, msg in rosout_msgs:
        text = msg.msg
        lower = text.lower()
        if ("rtde" in lower and "stale" in lower and "fault" in lower) or (
                "driver health fault latched" in lower):
            events.append(IncidentEvent(t_ns, "driver_fault", "error", text,
                                        f"/rosout:{msg.name}"))
    events.sort(key=lambda event: event.t_ns)
    return events


def analyze_driver_health(health_msgs, rosout_msgs, expected_rtde_rate_hz=None,
                          rtde_state_max_age_s=None):
    checks = []
    events = _fault_events(health_msgs, rosout_msgs)
    if not health_msgs:
        checks.append(_check(
            "driver_health_evidence", "Driver health evidence",
            CheckState.UNKNOWN,
            "No /aubo_driver/health messages; using legacy logs where possible"))
        if events:
            checks.append(_check(
                "driver_fault_free", "Driver remained fault-free",
                CheckState.FAIL, events[0].summary, t_ns=events[0].t_ns,
                evidence=[events[0].source]))
            checks.append(_check(
                "driver_fault_detected", "Injected/observed driver fault detected",
                CheckState.PASS, events[0].summary, t_ns=events[0].t_ns,
                evidence=[events[0].source]))
        else:
            checks.append(_check(
                "driver_fault_free", "Driver remained fault-free",
                CheckState.UNKNOWN,
                "Legacy logs contain no fault, but absence of the health topic "
                "cannot prove health"))
            checks.append(_check(
                "driver_fault_detected", "Injected/observed driver fault detected",
                CheckState.UNKNOWN, "No typed health evidence or legacy fault log"))
        return checks, events

    sequences = [int(msg.status_sequence) for _, msg in health_msgs]
    gaps = [(sequences[i - 1], sequences[i]) for i in range(1, len(sequences))
            if sequences[i] != sequences[i - 1] + 1]
    heartbeat_gaps = [health_msgs[i][0] - health_msgs[i - 1][0]
                      for i in range(1, len(health_msgs))]
    max_gap_ns = max(heartbeat_gaps, default=0)
    first_sequence = sequences[0]
    if (len(health_msgs) < 2 or first_sequence != 0 or gaps or
            max_gap_ns > 500_000_000):
        details = []
        if len(health_msgs) < 2:
            details.append("fewer than two health heartbeats")
        if first_sequence != 0:
            details.append(f"recording starts at status_sequence {first_sequence}")
        if gaps:
            details.append(f"{len(gaps)} status-sequence gap(s)")
        if max_gap_ns > 500_000_000:
            details.append(f"maximum heartbeat gap {max_gap_ns / 1e6:.1f} ms")
        checks.append(_check(
            "driver_health_evidence", "Driver health evidence",
            CheckState.UNKNOWN, "; ".join(details),
            evidence=[f"first status_sequence={first_sequence}",
                      f"messages={len(health_msgs)}"]))
    else:
        checks.append(_check(
            "driver_health_evidence", "Driver health evidence", CheckState.PASS,
            f"{len(health_msgs)} contiguous messages; maximum heartbeat gap "
            f"{max_gap_ns / 1e6:.1f} ms",
            evidence=[f"first status_sequence={first_sequence}"]))

    counter_names = ("source_sequence", "callback_count", "valid_sample_count")
    regressions = []
    last_by_activation = {}
    for _, msg in health_msgs:
        activation = int(msg.activation_sequence)
        previous = last_by_activation.get(activation)
        current = tuple(int(getattr(msg, name)) for name in counter_names)
        if previous is not None:
            for name, before, after in zip(counter_names, previous, current):
                if after < before:
                    regressions.append((activation, name, before, after))
        last_by_activation[activation] = current

    malformed = max((int(msg.malformed_position_count) +
                     int(msg.malformed_velocity_count) +
                     int(msg.parser_exception_count)
                     for _, msg in health_msgs), default=0)
    source_stalls = []
    for index in range(1, len(health_msgs)):
        previous_t, previous = health_msgs[index - 1]
        current_t, current = health_msgs[index]
        if (int(previous.activation_sequence) == int(current.activation_sequence)
                and int(previous.state) == 1 and int(current.state) == 1
                and previous.writes_enabled and current.writes_enabled
                and int(previous.source_sequence) == int(current.source_sequence)
                and current_t - previous_t > 50_000_000):
            source_stalls.append((current_t, int(current.source_sequence)))
    stale_ages = []
    if rtde_state_max_age_s is not None:
        stale_ages = [(t_ns, float(msg.source_age_sec)) for t_ns, msg in health_msgs
                      if int(msg.state) == 1 and
                      (not math.isfinite(msg.source_age_sec) or
                       float(msg.source_age_sec) > rtde_state_max_age_s)]
    rate_mismatches = []
    if expected_rtde_rate_hz is not None:
        rate_mismatches = [
            (t_ns, int(msg.requested_rate_hz)) for t_ns, msg in health_msgs
            if int(msg.requested_rate_hz) != int(expected_rtde_rate_hz)]
    if (events or malformed or regressions or source_stalls or stale_ages or
            rate_mismatches):
        parts = []
        if events:
            parts.append(events[0].summary)
        if malformed:
            parts.append(f"{malformed} malformed/parser callback(s)")
        if regressions:
            parts.append(f"{len(regressions)} counter regression(s)")
        if source_stalls:
            parts.append(f"{len(source_stalls)} active source-sequence stall(s)")
        if stale_ages:
            parts.append(
                f"{len(stale_ages)} active source age violation(s), limit "
                f"{rtde_state_max_age_s:.3f} s")
        if rate_mismatches:
            parts.append(
                f"requested RTDE rate does not match expected "
                f"{int(expected_rtde_rate_hz)} Hz")
        checks.append(_check(
            "driver_fault_free", "Driver remained fault-free", CheckState.FAIL,
            "; ".join(parts), t_ns=events[0].t_ns if events else None))
    else:
        latest = health_msgs[-1][1]
        checks.append(_check(
            "driver_fault_free", "Driver remained fault-free", CheckState.PASS,
            f"No fault or malformed callback; RTDE requested/observed "
            f"{int(latest.requested_rate_hz)}/{latest.observed_rate_hz:.1f} Hz; "
            f"interval p99/max {latest.interval_p99_ms:.3f}/"
            f"{latest.interval_max_ms:.3f} ms"))

    if events:
        checks.append(_check(
            "driver_fault_detected", "Injected/observed driver fault detected",
            CheckState.PASS, events[0].summary, t_ns=events[0].t_ns))
    else:
        checks.append(_check(
            "driver_fault_detected", "Injected/observed driver fault detected",
            CheckState.NOT_APPLICABLE, "No driver fault occurred in this bag"))
    return checks, events


def _joint_state_issues(joint_msgs):
    issues = []
    first_bad_ns = None
    for t_ns, msg in joint_msgs:
        names = list(msg.name)
        position = list(msg.position)
        velocity = list(msg.velocity)
        reason = None
        if not names or len(position) != len(names):
            reason = (f"malformed position vector names={len(names)} "
                      f"positions={len(position)}")
        elif not all(math.isfinite(value) for value in position):
            reason = "non-finite joint position"
        elif velocity and len(velocity) != len(names):
            reason = (f"malformed velocity vector names={len(names)} "
                      f"velocities={len(velocity)}")
        elif velocity and not all(math.isfinite(value) for value in velocity):
            reason = "non-finite joint velocity"
        if reason:
            issues.append((t_ns, reason))
            first_bad_ns = t_ns if first_bad_ns is None else first_bad_ns
    return issues, first_bad_ns


def _controller_freeze_after(ctrl_msgs, fault_t_ns):
    samples = []
    for t_ns, msg in ctrl_msgs:
        if fault_t_ns is not None and t_ns < fault_t_ns:
            continue
        names = list(msg.joint_names)
        ref = list(_point_positions(msg, "reference", "desired"))
        act = list(_point_positions(msg, "feedback", "actual"))
        if len(names) != len(ref) or len(names) != len(act):
            continue
        if not all(math.isfinite(v) for v in ref + act):
            continue
        samples.append((t_ns, np.asarray(ref), np.asarray(act)))
    if len(samples) < 3:
        return None
    base_ref = samples[0][1]
    base_act = samples[0][2]
    ref_motion = max(float(np.max(np.abs(ref - base_ref)))
                     for _, ref, _ in samples)
    actual_motion = max(float(np.max(np.abs(act - base_act)))
                        for _, _, act in samples)
    divergence = max(float(np.max(np.abs(ref - act)))
                     for _, ref, act in samples)
    return {
        "samples": len(samples), "ref_motion_rad": ref_motion,
        "actual_motion_rad": actual_motion, "divergence_rad": divergence,
        "frozen": ref_motion > 1e-3 and actual_motion < 1e-6,
    }


def analyze_feedback(joint_msgs, ctrl_msgs, fault_events, health_msgs):
    checks = []
    events = []
    if not joint_msgs:
        checks.append(_check("feedback_finite", "Feedback vectors are finite",
                             CheckState.UNKNOWN, "No /joint_states messages"))
    else:
        issues, first_bad_ns = _joint_state_issues(joint_msgs)
        if issues:
            by_reason = {}
            for _, reason in issues:
                by_reason[reason] = by_reason.get(reason, 0) + 1
            summary = ", ".join(f"{count} {reason} sample(s)"
                                for reason, count in by_reason.items())
            checks.append(_check(
                "feedback_finite", "Feedback vectors are finite",
                CheckState.FAIL, summary, t_ns=first_bad_ns))
            events.append(IncidentEvent(first_bad_ns, "invalid_feedback", "error",
                                        issues[0][1], "/joint_states"))
        else:
            checks.append(_check(
                "feedback_finite", "Feedback vectors are finite", CheckState.PASS,
                f"{len(joint_msgs)} joint-state samples are structurally valid "
                "and finite"))

    fault_t_ns = fault_events[0].t_ns if fault_events else None
    freeze = _controller_freeze_after(ctrl_msgs, fault_t_ns)
    source_progress = None
    if health_msgs:
        relevant = [(t, m) for t, m in health_msgs
                    if fault_t_ns is None or t >= fault_t_ns]
        if len(relevant) >= 2:
            source_progress = (int(relevant[-1][1].source_sequence) >
                               int(relevant[0][1].source_sequence))
    if freeze and freeze["frozen"]:
        summary = (f"Controller reference moved {math.degrees(freeze['ref_motion_rad']):.3f} "
                   f"deg while measured state moved only "
                   f"{math.degrees(freeze['actual_motion_rad']):.6f} deg over "
                   f"{freeze['samples']} samples")
        checks.append(_check(
            "feedback_not_frozen", "Measured feedback is not frozen",
            CheckState.FAIL, summary, confidence="derived", t_ns=fault_t_ns))
        events.append(IncidentEvent(fault_t_ns or ctrl_msgs[0][0],
                                    "frozen_feedback", "error", summary,
                                    "/joint_trajectory_controller/controller_state"))
    elif source_progress is False:
        checks.append(_check(
            "feedback_not_frozen", "Measured feedback is not frozen",
            CheckState.FAIL, "Driver source sequence stopped progressing",
            t_ns=fault_t_ns))
    elif health_msgs and not fault_events:
        checks.append(_check(
            "feedback_not_frozen", "Measured feedback is not frozen",
            CheckState.PASS,
            "Health source sequence is available and no frozen/faulted state was detected"))
    elif fault_events and freeze is not None:
        checks.append(_check(
            "feedback_not_frozen", "Measured feedback is not frozen",
            CheckState.PASS,
            "Measured state continued changing after the fault marker",
            confidence="derived"))
    else:
        checks.append(_check(
            "feedback_not_frozen", "Measured feedback is not frozen",
            CheckState.UNKNOWN,
            "No typed source sequence and insufficient commanded motion to prove freshness"))
    return checks, events


def analyze_executor_behavior(rosout_msgs, fault_events):
    checks = []
    events = []
    command_starts = []
    endpoint_failures = []
    joint_fallbacks = []
    contact_abort = None
    retreat = None
    structured = False
    execution_failures = []
    controller_successes = []
    for t_ns, msg in rosout_msgs:
        if msg.name == "joint_trajectory_controller" and \
                "Goal reached, success" in msg.msg:
            controller_successes.append((t_ns, msg.msg))
        if msg.name != ROSOUT_NODE:
            continue
        text = msg.msg
        if COMMAND_START_RE.match(text):
            command_starts.append((t_ns, text))
        if "Endpoint failed to settle" in text or "One-shot endpoint" in text and "failed" in text:
            endpoint_failures.append((t_ns, text))
        if "Selected shortest bounded joint-space candidate" in text:
            joint_fallbacks.append((t_ns, text))
        if "Aborting with the pen down" in text:
            contact_abort = t_ns
        if "Pen retreated to hover height" in text:
            retreat = (t_ns, "succeeded")
        if "refusing an autonomous retreat" in text:
            retreat = (t_ns, "safely refused from unknown state")
        if text.startswith("motion_outcome "):
            structured = True
        if "Execution reported failure" in text:
            execution_failures.append((t_ns, text))

    if fault_events:
        fault_t_ns = fault_events[0].t_ns
        after = [(t, text) for t, text in command_starts if t > fault_t_ns]
        if after:
            checks.append(_check(
                "no_command_after_fault", "No command starts after a health fault",
                CheckState.FAIL, f"{len(after)} command(s) started after the first fault",
                confidence="direct", t_ns=after[0][0], evidence=[after[0][1]]))
            events.append(IncidentEvent(after[0][0], "post_fault_command", "error",
                                        after[0][1], "/rosout:painting_executor"))
        else:
            checks.append(_check(
                "no_command_after_fault", "No command starts after a health fault",
                CheckState.PASS, "No later painting_executor command start was recorded"))
    else:
        checks.append(_check(
            "no_command_after_fault", "No command starts after a health fault",
            CheckState.PASS, "No health fault occurred; no post-fault command exists"))

    forbidden = []
    for endpoint_t, endpoint_text in endpoint_failures:
        candidate = next(((t, text) for t, text in joint_fallbacks
                          if t > endpoint_t), None)
        if candidate:
            forbidden.append((endpoint_t, endpoint_text, candidate))
    if forbidden:
        t_ns, endpoint_text, fallback = forbidden[0]
        checks.append(_check(
            "no_post_execution_fallback",
            "No joint-space fallback after post-execution failure",
            CheckState.FAIL,
            "A joint-space candidate was selected after endpoint failure",
            confidence="derived", t_ns=fallback[0],
            evidence=[endpoint_text, fallback[1]]))
        events.append(IncidentEvent(fallback[0], "post_execution_fallback", "error",
                                    fallback[1], "/rosout:painting_executor"))
    elif endpoint_failures:
        checks.append(_check(
            "no_post_execution_fallback",
            "No joint-space fallback after post-execution failure",
            CheckState.PASS,
            "Endpoint failure was not followed by a joint-space candidate",
            confidence="derived"))
    else:
        checks.append(_check(
            "no_post_execution_fallback",
            "No joint-space fallback after post-execution failure",
            CheckState.PASS, "No post-execution endpoint failure occurred"))

    if contact_abort is None:
        checks.append(_check("bounded_recovery", "Contact recovery stayed bounded",
                             CheckState.PASS,
                             "No pen-down abort required contact recovery"))
    elif retreat:
        checks.append(_check(
            "bounded_recovery", "Contact recovery stayed bounded", CheckState.PASS,
            f"Straight retreat {retreat[1]}", t_ns=retreat[0]))
    else:
        checks.append(_check(
            "bounded_recovery", "Contact recovery stayed bounded",
            CheckState.UNKNOWN,
            "Pen-down abort was recorded without a conclusive retreat outcome",
            t_ns=contact_abort))

    checks.append(_check(
        "structured_motion_outcomes", "Typed motion outcomes are available",
        CheckState.PASS if structured else CheckState.NOT_APPLICABLE,
        ("Structured motion_outcome events were recorded" if structured else
         "Phase 3 structured motion outcomes are not implemented in this bag")))
    split_outcomes = []
    for failure_t, failure_text in execution_failures:
        success = next(((t, text) for t, text in controller_successes
                        if failure_t < t <= failure_t + 1_000_000_000), None)
        if success:
            split_outcomes.append((failure_t, failure_text, success))
    if split_outcomes:
        failure_t, failure_text, success = split_outcomes[0]
        checks.append(_check(
            "moveit_controller_consistency",
            "MoveIt and controller outcomes agree", CheckState.FAIL,
            f"MoveIt reported failure, then the controller reported success "
            f"{(success[0] - failure_t) / 1e6:.1f} ms later",
            confidence="derived", t_ns=failure_t,
            evidence=[failure_text, success[1]]))
        events.append(IncidentEvent(
            failure_t, "split_motion_outcome", "error",
            "MoveIt execution failure followed by controller success",
            "/rosout"))
    elif execution_failures:
        checks.append(_check(
            "moveit_controller_consistency",
            "MoveIt and controller outcomes agree", CheckState.UNKNOWN,
            "MoveIt reported execution failure without a structured controller result"))
    else:
        checks.append(_check(
            "moveit_controller_consistency",
            "MoveIt and controller outcomes agree", CheckState.PASS,
            "No MoveIt execution failure was recorded"))
    return checks, events


def analyze_servoj_runtime(rosout_msgs):
    servoj = parse_servoj_diag(rosout_msgs)
    if servoj is None:
        return [_check(
            "servoj_runtime", "ServoJ runtime diagnostics", CheckState.UNKNOWN,
            "No aubo_servoj_diag runtime reports were recorded")], []
    warn = servoj["warnings"]
    aggregate = servoj.get("aggregate")
    qf = (aggregate.get("qf_events", 0) if aggregate else 0) + \
        int(warn["queue_full"])
    non_ok = (aggregate.get("non_ok_rc", 0) if aggregate else 0)
    exceptions = (aggregate.get("exc", 0) if aggregate else 0)
    if qf or non_ok or exceptions or warn["rc"] or warn["fault_latched"]:
        summary = (f"queue-full {qf}, non-OK return codes {non_ok}, "
                   f"exceptions {exceptions}, return-code warnings {warn['rc']}, "
                   f"fault latched {'yes' if warn['fault_latched'] else 'no'}")
        event_t = next((t for t, msg in rosout_msgs
                        if msg.name == SERVOJ_NODE and
                        ("queue-full" in msg.msg or "servoj_rc" in msg.msg or
                         "fault" in msg.msg)), None)
        events = [IncidentEvent(event_t, "servoj_runtime_failure", "error",
                                summary, "/rosout:aubo_servoj_diag")]
        return [_check("servoj_runtime", "ServoJ runtime diagnostics",
                       CheckState.FAIL, summary, t_ns=event_t)], events
    reports = len(servoj.get("reports", []))
    return [_check(
        "servoj_runtime", "ServoJ runtime diagnostics", CheckState.PASS,
        f"{reports} report window(s), no queue-full, non-OK return, exception, "
        "or latched fault")], []


SERVOJ_CALL_COLUMNS = [
    "seq", "t_ros_s", "t_wall_s", "t_call_ros_s",
    "cmd0", "cmd1", "cmd2", "cmd3", "cmd4", "cmd5",
    "servoj_t_s", "rpc_ms", "rc", "qf_retries", "dropped", "exc",
]
SERVOJ_FOOTER_RE = re.compile(
    r"^# end last_seq=(\d+) rows=(\d+) dropped=(\d+) "
    r"status=(complete|incomplete)$")


def read_servoj_calls_csv(path):
    path = Path(path)
    result = {"path": str(path), "rows": [], "footer": None, "issues": [],
              "clock_step_indices": []}
    partial = Path(str(path) + ".partial")
    if partial.exists():
        result["issues"].append(f"partial telemetry file remains: {partial}")
    if not path.is_file():
        result["issues"].append(f"final telemetry file is missing: {path}")
        return result
    lines = path.read_text().splitlines()
    if not lines:
        result["issues"].append("telemetry file is empty")
        return result
    if lines[0].split(",") != SERVOJ_CALL_COLUMNS:
        result["issues"].append("unexpected ServoJ CSV schema")
        return result
    footer_match = SERVOJ_FOOTER_RE.match(lines[-1])
    if footer_match:
        result["footer"] = {
            "last_seq": int(footer_match.group(1)),
            "rows": int(footer_match.group(2)),
            "dropped": int(footer_match.group(3)),
            "status": footer_match.group(4),
        }
        data_lines = lines[1:-1]
    else:
        result["issues"].append("missing or malformed ServoJ CSV footer")
        data_lines = lines[1:]
    reader = csv.DictReader([lines[0]] + data_lines)
    try:
        for raw in reader:
            row = {"seq": int(raw["seq"]), "rc": int(raw["rc"]),
                   "qf_retries": int(raw["qf_retries"]),
                   "dropped": int(raw["dropped"]), "exc": int(raw["exc"])}
            for name in SERVOJ_CALL_COLUMNS[1:12]:
                row[name] = float(raw[name])
            result["rows"].append(row)
    except (TypeError, ValueError, KeyError) as exc:
        result["issues"].append(f"malformed ServoJ CSV row: {exc}")
        return result
    rows = result["rows"]
    if not rows:
        result["issues"].append("ServoJ CSV contains no data rows")
        return result
    seq = [row["seq"] for row in rows]
    if seq != list(range(len(rows))):
        result["issues"].append("ServoJ sequence is not contiguous from zero")
    footer = result["footer"]
    if footer:
        if footer["status"] != "complete" or footer["dropped"] != 0:
            result["issues"].append(
                f"footer status={footer['status']} dropped={footer['dropped']}")
        if footer["rows"] != len(rows) or footer["last_seq"] != rows[-1]["seq"]:
            result["issues"].append("footer row/sequence counts do not match data")
    for index, row in enumerate(rows):
        numeric = [row[name] for name in SERVOJ_CALL_COLUMNS[1:12]]
        if not all(math.isfinite(value) for value in numeric):
            result["issues"].append(f"row {index} contains a non-finite value")
            break
    intervals = np.diff([row["t_wall_s"] for row in rows])
    result["intervals_s"] = intervals.tolist()
    if len(intervals) and np.any(intervals <= 0.0):
        result["issues"].append("steady-clock ServoJ timestamps are not increasing")
    offsets = np.asarray([row["t_call_ros_s"] - row["t_wall_s"]
                          for row in rows])
    if len(offsets) > 1:
        offset_delta = np.diff(offsets)
        result["clock_step_indices"] = [
            int(i + 1) for i, value in enumerate(offset_delta)
            if abs(value) > 0.010]
        result["max_clock_offset_step_s"] = float(np.max(np.abs(offset_delta)))
    else:
        result["max_clock_offset_step_s"] = 0.0
    return result


def analyze_servoj_calls(
        path, expected_rate_hz=DEFAULT_EXPECTED_RATE_HZ,
        expected_servoj_t_s=DEFAULT_EXPECTED_SERVOJ_T_S):
    if path is None:
        return [_check(
            "servoj_telemetry", "Full-rate ServoJ telemetry",
            CheckState.UNKNOWN, "No --servoj-calls-csv was supplied")], [], None
    telemetry = read_servoj_calls_csv(path)
    rows = telemetry["rows"]
    operational = []
    for row in rows:
        if row["rc"] != 0 or row["qf_retries"] != 0 or row["dropped"] or row["exc"]:
            operational.append(row)
    profile_issues = []
    if rows and expected_servoj_t_s is not None:
        if any(not math.isclose(row["servoj_t_s"], expected_servoj_t_s,
                                rel_tol=0.0, abs_tol=1e-9) for row in rows):
            profile_issues.append(
                f"servoj_t_s does not match expected {expected_servoj_t_s}")
    intervals = telemetry.get("intervals_s", [])
    effective_rate = 1.0 / float(np.mean(intervals)) if intervals else None
    if expected_rate_hz is not None and effective_rate is not None:
        if effective_rate < 0.95 * expected_rate_hz:
            profile_issues.append(
                f"effective rate {effective_rate:.1f} Hz is below 95% of "
                f"expected {expected_rate_hz:.1f} Hz")
    issues = telemetry["issues"] + profile_issues
    if operational:
        first = operational[0]
        issues.append(
            f"{len(operational)} operational failure row(s); first seq "
            f"{first['seq']} rc={first['rc']} retries={first['qf_retries']} "
            f"dropped={first['dropped']} exc={first['exc']}")
    if issues:
        check = _check("servoj_telemetry", "Full-rate ServoJ telemetry",
                       CheckState.FAIL, "; ".join(issues))
    else:
        check = _check(
            "servoj_telemetry", "Full-rate ServoJ telemetry", CheckState.PASS,
            f"{len(rows)} contiguous rows, {effective_rate:.1f} Hz monotonic cadence, "
            "zero operational or telemetry failures")
    events = []
    if operational:
        events.append(IncidentEvent(
            int(operational[0]["t_call_ros_s"] * 1e9), "servoj_failure", "error",
            check.summary, str(path)))
    return [check], events, telemetry


def analyze_clock(evidence, rosout_msgs, telemetry):
    events = []
    direct = []
    for t_ns, msg in rosout_msgs:
        lower = msg.msg.lower()
        if ("moved backwards in time" in lower or
                "moved backward in time" in lower):
            direct.append((t_ns, msg.msg, msg.name))
    regressions = []
    large_gaps = []
    for topic in ("/joint_states",
                  "/joint_trajectory_controller/controller_state"):
        messages = evidence["messages"].get(topic, [])
        stamps = [_message_stamp_ns(msg, t_ns) for t_ns, msg in messages]
        for index in range(1, len(stamps)):
            delta = stamps[index] - stamps[index - 1]
            if delta < 0:
                regressions.append((topic, messages[index][0], delta))
            elif delta > 100_000_000:
                large_gaps.append((topic, messages[index][0], delta))
    clock_steps = telemetry.get("clock_step_indices", []) if telemetry else []
    if direct or regressions or clock_steps:
        if direct:
            t_ns, text, node = direct[0]
            summary = f"{len(direct)} backward-time log(s); first: {text}"
        elif regressions:
            topic, t_ns, delta = regressions[0]
            summary = f"Embedded timestamp regressed {abs(delta) / 1e6:.3f} ms on {topic}"
        else:
            t_ns = int(telemetry["rows"][clock_steps[0]]["t_call_ros_s"] * 1e9)
            summary = ("System-minus-steady clock offset changed by up to "
                       f"{telemetry['max_clock_offset_step_s'] * 1000.0:.3f} ms")
        events.append(IncidentEvent(t_ns, "clock_discontinuity", "error", summary,
                                    "clock correlation"))
        check = _check("clock_stable", "Wall clock remained stable",
                       CheckState.FAIL, summary, t_ns=t_ns)
    elif telemetry:
        check = _check(
            "clock_stable", "Wall clock remained stable", CheckState.PASS,
            "No timestamp regression or system-versus-steady offset step detected")
    elif large_gaps:
        max_gap = max(value for _, _, value in large_gaps)
        check = _check(
            "clock_stable", "Wall clock remained stable", CheckState.UNKNOWN,
            f"No monotonic clock artifact; simultaneous ROS timestamp gaps reach "
            f"{max_gap / 1e3:.0f} us and cannot distinguish clock correction from delivery")
    else:
        check = _check(
            "clock_stable", "Wall clock remained stable", CheckState.UNKNOWN,
            "No full-rate monotonic/system clock correlation artifact was supplied")
    return [check], events


def _capability_checks(manifest):
    checks = []
    checks.append(_check(
        "controller_hardening", "Patched controller evidence",
        CheckState.NOT_APPLICABLE,
        "Controller hardening is not yet represented by analyzer-readable evidence"))
    if manifest is None:
        checks.append(_check(
            "runtime_provenance", "Runtime provenance manifest",
            CheckState.NOT_APPLICABLE,
            "Phase 6 runtime provenance is not implemented or no manifest was supplied"))
    else:
        path = Path(manifest)
        try:
            with open(path) as stream:
                data = yaml.safe_load(stream)
            if not isinstance(data, dict):
                raise ValueError("manifest root is not a mapping")
        except (OSError, ValueError, yaml.YAMLError) as exc:
            checks.append(_check(
                "runtime_provenance", "Runtime provenance manifest",
                CheckState.UNKNOWN, f"Cannot validate {manifest}: {exc}"))
        else:
            checks.append(_check(
                "runtime_provenance", "Runtime provenance manifest",
                CheckState.PASS, f"Manifest loaded from {manifest}"))
    return checks


def _overall(checks, mode, gate):
    by_key = {check.key: check for check in checks}
    if gate:
        required = GATE_REQUIREMENTS[gate]
        selected = []
        for key in required:
            selected.append(by_key.get(key, _check(
                key, key.replace("_", " ").title(), CheckState.UNKNOWN,
                "Required check was not produced")))
        if any(check.state == CheckState.FAIL for check in selected):
            return "FAIL", 1
        if any(check.state in (CheckState.UNKNOWN, CheckState.NOT_APPLICABLE)
               for check in selected):
            return "INCOMPLETE", 3
        return "PASS", 0
    failures = [check for check in checks if check.state == CheckState.FAIL]
    return ("FAIL", 1) if failures else ("DEBUG", 0)


def analyze_remediation(bag_dir, mode="debug", gate=None, servoj_calls_csv=None,
                        expected_rate_hz=DEFAULT_EXPECTED_RATE_HZ,
                        expected_servoj_t_s=DEFAULT_EXPECTED_SERVOJ_T_S,
                        expected_rtde_rate_hz=None,
                        rtde_state_max_age_s=None, manifest=None):
    canonical_topics = [
        DRIVER_HEALTH_TOPIC, "/joint_states",
        "/joint_trajectory_controller/controller_state", "/rosout",
        "/parameter_events", "/robot_description",
    ]
    evidence = read_bag_evidence(bag_dir, canonical_topics)
    messages = evidence["messages"]
    rosout = messages.get("/rosout", [])
    checks = []
    events = []

    health_checks, fault_events = analyze_driver_health(
        messages.get(DRIVER_HEALTH_TOPIC, []), rosout,
        expected_rtde_rate_hz=expected_rtde_rate_hz,
        rtde_state_max_age_s=rtde_state_max_age_s)
    checks.extend(health_checks)
    events.extend(fault_events)
    feedback_checks, feedback_events = analyze_feedback(
        messages.get("/joint_states", []),
        messages.get("/joint_trajectory_controller/controller_state", []),
        fault_events, messages.get(DRIVER_HEALTH_TOPIC, []))
    checks.extend(feedback_checks)
    events.extend(feedback_events)
    executor_checks, executor_events = analyze_executor_behavior(
        rosout, fault_events)
    checks.extend(executor_checks)
    events.extend(executor_events)
    runtime_checks, runtime_events = analyze_servoj_runtime(rosout)
    checks.extend(runtime_checks)
    events.extend(runtime_events)
    servoj_checks, servoj_events, telemetry = analyze_servoj_calls(
        servoj_calls_csv, expected_rate_hz, expected_servoj_t_s)
    checks.extend(servoj_checks)
    events.extend(servoj_events)
    clock_checks, clock_events = analyze_clock(evidence, rosout, telemetry)
    checks.extend(clock_checks)
    events.extend(clock_events)
    checks.extend(_capability_checks(manifest))
    events.sort(key=lambda event: (event.t_ns is None, event.t_ns or 0))
    overall, exit_code = _overall(checks, mode, gate)
    return RemediationReport(
        str(bag_dir), mode, gate, evidence["start_ns"], evidence["end_ns"],
        {name: count for name, count in evidence["topic_counts"].items()
         if count}, checks, events, overall, exit_code)


def render_remediation_summary(report):
    lines = ["# Remediation bag analysis", "",
             f"Result: **{report.overall}**",
             f"Mode: `{report.mode}`" +
             (f", gate: `{report.gate}`" if report.gate else ""), "",
             "## Evidence checks", "",
             "| state | check | finding |", "| --- | --- | --- |"]
    for check in report.checks:
        lines.append(f"| {check.state.value} | {check.title} | {check.summary} |")
    lines.extend(["", "## Incident timeline", ""])
    if not report.events:
        lines.append("No remediation-relevant incident was detected.")
    else:
        for event in report.events:
            stamp = "n/a" if event.t_ns is None else f"{event.t_ns / 1e9:.9f}"
            lines.append(f"- `{stamp}` [{event.severity}] {event.category}: "
                         f"{event.summary} (`{event.source}`)")
    lines.extend(["", "## Topic inventory", ""])
    for topic, count in sorted(report.topic_counts.items()):
        lines.append(f"- `{topic}`: {count}")
    return "\n".join(lines)


def write_remediation_json(path, report):
    with open(path, "w") as stream:
        json.dump(asdict(report), stream, indent=2)
        stream.write("\n")


def write_events_csv(path, events):
    with open(path, "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["t_ns", "category", "severity", "summary", "source"])
        for event in events:
            writer.writerow([event.t_ns, event.category, event.severity,
                             event.summary, event.source])


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
    # reversal/curved references yield a meaningful
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
    fixture. A cycle spans two reversals of the dominant
    tangential axis; falls back to the whole segment under one full cycle."""
    xy = np.asarray(tangential_xy_mm, dtype=float)
    ne = np.asarray(normal_err_mm, dtype=float)
    n = len(ne)
    whole = float(ne.max() - ne.min()) if n else 0.0
    if n < 3:
        return {"n_cycles": 0, "pp_mean_mm": whole, "pp_max_mm": whole}
    # Pick the OSCILLATING tangential axis (the one that actually reverses),
    # not the axis with the greatest total travel. An oscillating curve may
    # advance ~90 mm monotonically in X while oscillating ~48 mm in Y, so
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
    """Historical pooled command-to-feedback and canvas-normal summary."""
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
    # Preserve the historical screen's missing-delay behavior for comparison.
    # Current acceptance rules are documented separately and do not use this
    # pooled result as the run verdict.
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
    lines.append(f"Historical global tracking screen: {status} ({detail})")
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
    # The configured rate is a mandatory timing input: without a servoj_config
    # line the bag cannot prove it ran at the intended rate, so the screen reads
    # INCOMPLETE
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
    lines.append(f"ServoJ timing screen: {status} ({detail})")
    lines.append(
        "  (Historical joint-delay targets <30 ms median / <50 ms p95 are "
        "reported separately from controller_state cross-correlation.)")
    return lines


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def analyze(bag_dir, canvas_file, calibration_file, urdf=None,
            plane_bias_mm=1.0, base_frame="base_link", ee_frame="ee_link"):
    """Historical geometry analysis; returns (metrics, rates, servoj)."""
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
    ap.add_argument("--mode", choices=["debug"], default="debug",
                    help="debug reports all available remediation evidence")
    ap.add_argument("--gate", choices=sorted(GATE_REQUIREMENTS),
                    help="apply the selected strict remediation gate")
    ap.add_argument("--servoj-calls-csv",
                    help="driver full-rate ServoJ call CSV input")
    ap.add_argument("--expected-rate-hz", type=float,
                    default=DEFAULT_EXPECTED_RATE_HZ,
                    help="expected controller/ServoJ call rate (default: 125)")
    ap.add_argument("--expected-servoj-t-s", type=float,
                    default=DEFAULT_EXPECTED_SERVOJ_T_S,
                    help="expected ServoJ interpolation time (default: 0.008)")
    ap.add_argument("--expected-rtde-rate-hz", type=int,
                    help="expected requested RTDE source rate")
    ap.add_argument("--rtde-state-max-age-s", type=float,
                    help="active RTDE source-age limit used for this run")
    ap.add_argument("--manifest",
                    help="runtime provenance manifest (capability marker)")
    ap.add_argument("--json", help="write structured remediation report")
    ap.add_argument("--events-csv", help="write incident timeline CSV")
    ap.add_argument("--canvas-file",
                    help="optional teach_canvas YAML for geometry diagnostics")
    ap.add_argument("--calibration-file",
                    help="optional hardware YAML for geometry diagnostics")
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

    if bool(args.canvas_file) != bool(args.calibration_file):
        ap.error("--canvas-file and --calibration-file must be supplied together")
    if args.expected_rate_hz is not None and args.expected_rate_hz <= 0.0:
        ap.error("--expected-rate-hz must be positive")
    if args.expected_servoj_t_s is not None and args.expected_servoj_t_s <= 0.0:
        ap.error("--expected-servoj-t-s must be positive")
    if args.expected_rtde_rate_hz is not None and args.expected_rtde_rate_hz <= 0:
        ap.error("--expected-rtde-rate-hz must be positive")
    if args.rtde_state_max_age_s is not None and \
            args.rtde_state_max_age_s <= 0.0:
        ap.error("--rtde-state-max-age-s must be positive")

    report = analyze_remediation(
        args.bag_dir, mode=args.mode, gate=args.gate,
        servoj_calls_csv=args.servoj_calls_csv,
        expected_rate_hz=args.expected_rate_hz,
        expected_servoj_t_s=args.expected_servoj_t_s,
        expected_rtde_rate_hz=args.expected_rtde_rate_hz,
        rtde_state_max_age_s=args.rtde_state_max_age_s,
        manifest=args.manifest)
    if args.json:
        write_remediation_json(args.json, report)
    if args.events_csv:
        write_events_csv(args.events_csv, report.events)
    print(render_remediation_summary(report))

    if args.canvas_file:
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
        print("\n" + render_summary(metrics_list, rates, servoj))
    elif args.csv or args.servoj_csv or args.urdf:
        ap.error("historical tracking outputs require canvas and calibration files")
    return report.exit_code


if __name__ == "__main__":
    sys.exit(main())
