"""Tests for scripts/analyze_tracking_bag.py against synthetic rosbag2 bags.

The July 22 baseline bags live on the robot host; these tests build small
synthetic bags with known geometry so the analysis math, sign conventions,
segmentation, and no-publish guarantee gate locally in CI.
"""

import importlib.util
import math
from pathlib import Path

import numpy as np
import yaml

import rosbag2_py
from rclpy.serialization import serialize_message
from control_msgs.msg import JointTrajectoryControllerState
from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState
from std_msgs.msg import String

_SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def _load(name):
    spec = importlib.util.spec_from_file_location(name,
                                                  _SCRIPTS / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


analyze_tracking_bag = _load("analyze_tracking_bag")
teach_canvas = _load("teach_canvas")

# Prismatic gantry: joint positions ARE the tip base-frame coordinates, so
# every expected value in these tests is exact.
GANTRY_URDF = """
<robot name="gantry">
  <link name="base_link"/><link name="lx"/><link name="ly"/>
  <link name="ee_link"/>
  <joint name="jx" type="prismatic">
    <parent link="base_link"/><child link="lx"/><axis xyz="1 0 0"/>
  </joint>
  <joint name="jy" type="prismatic">
    <parent link="lx"/><child link="ly"/><axis xyz="0 1 0"/>
  </joint>
  <joint name="jz" type="prismatic">
    <parent link="ly"/><child link="ee_link"/><axis xyz="0 0 1"/>
  </joint>
</robot>
"""

REVOLUTE_URDF = """
<robot name="arm">
  <link name="base_link"/><link name="l1"/><link name="ee_link"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/><child link="l1"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="tip" type="fixed">
    <origin xyz="0.5 0 0"/>
    <parent link="l1"/><child link="ee_link"/>
  </joint>
</robot>
"""

S = 1_000_000_000  # ns per second


def _log(text):
    m = Log()
    m.name = "painting_executor"
    m.msg = text
    return m


def _state(ref_xyz, act_xyz):
    m = JointTrajectoryControllerState()
    m.joint_names = ["jx", "jy", "jz"]
    m.reference.positions = [float(v) for v in ref_xyz]
    m.feedback.positions = [float(v) for v in act_xyz]
    return m


def _write_bag(bag_dir, messages):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    seen = {}
    for _t, topic, type_str, _msg in messages:
        if topic not in seen:
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type=type_str, serialization_format="cdr"))
            seen[topic] = True
    for t_ns, topic, _type_str, msg in sorted(messages, key=lambda x: x[0]):
        writer.write(topic, serialize_message(msg), t_ns)
    del writer


def _synthetic_bag(bag_dir):
    """Two-command bag: +Y stroke with +0.5 mm into-paper error, then an
    error-free -X stroke. Canvas frame == base frame (identity quat)."""
    msgs = []
    msgs.append((0, "/robot_description", "std_msgs/msg/String",
                 String(data=GANTRY_URDF)))
    msgs.append((1 * S, "/rosout", "rcl_interfaces/msg/Log",
                 _log("[1/2] paint_stroke (row_a)")))
    msgs.append((2 * S, "/rosout", "rcl_interfaces/msg/Log",
                 _log("[2/2] paint_stroke (row_b)")))
    msgs.append((3 * S, "/rosout", "rcl_interfaces/msg/Log",
                 _log("Painting finished (2 commands)")))
    n = 51
    for i in range(n):
        # Segment 1: +Y 0 -> 30 mm over 0.5 s, actual 0.5 mm INTO the paper.
        y = 0.030 * i / (n - 1)
        ref = [0.0, y, 0.0]
        act = [0.0, y, 0.0005]
        msgs.append((1 * S + int(0.5 * S * i / (n - 1)),
                     "/joint_trajectory_controller/controller_state",
                     "control_msgs/msg/JointTrajectoryControllerState",
                     _state(ref, act)))
        # Segment 2: -X 50 -> 20 mm, no error.
        x = 0.050 - 0.030 * i / (n - 1)
        msgs.append((2 * S + int(0.5 * S * i / (n - 1)),
                     "/joint_trajectory_controller/controller_state",
                     "control_msgs/msg/JointTrajectoryControllerState",
                     _state([x, 0.0, 0.0], [x, 0.0, 0.0])))
    for i in range(20):
        js = JointState()
        js.name = ["jx", "jy", "jz"]
        js.position = [0.0, 0.0, 0.0]
        msgs.append((1 * S + int(0.1 * S * i), "/joint_states",
                     "sensor_msgs/msg/JointState", js))
    _write_bag(bag_dir, msgs)


def _write_yaml(path, params):
    with open(path, "w") as f:
        yaml.safe_dump(
            {"painting_executor": {"ros__parameters": params}}, f)


def _fixture_files(tmp_path):
    bag_dir = tmp_path / "bag"
    _synthetic_bag(bag_dir)
    canvas = tmp_path / "canvas.yaml"
    _write_yaml(canvas, {
        "canvas_origin_xyz": [0.0, 0.0, 0.0],
        "canvas_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
    })
    calib = tmp_path / "calib.yaml"
    _write_yaml(calib, {
        "tool_offset_xyz": [0.0, 0.0, 0.0],
        "tool_offset_rpy": [0.0, 0.0, 0.0],
    })
    return bag_dir, canvas, calib


def test_fk_gantry_maps_joints_to_tip():
    chain = analyze_tracking_bag.parse_urdf_chain(GANTRY_URDF, "base_link",
                                                  "ee_link")
    tip = analyze_tracking_bag.tip_position(
        chain, np.eye(4), {"jx": 0.1, "jy": -0.2, "jz": 0.3})
    np.testing.assert_allclose(tip, [0.1, -0.2, 0.3], atol=1e-12)


def test_fk_revolute_rotates_fixed_tip():
    chain = analyze_tracking_bag.parse_urdf_chain(REVOLUTE_URDF, "base_link",
                                                  "ee_link")
    tip = analyze_tracking_bag.tip_position(chain, np.eye(4),
                                            {"j1": math.pi / 2.0})
    np.testing.assert_allclose(tip, [0.0, 0.5, 0.0], atol=1e-12)


def test_tool_offset_applies_in_ee_frame():
    chain = analyze_tracking_bag.parse_urdf_chain(REVOLUTE_URDF, "base_link",
                                                  "ee_link")
    tool_T = analyze_tracking_bag.make_transform([0.1, 0.0, 0.0],
                                                 [0.0, 0.0, 0.0])
    tip = analyze_tracking_bag.tip_position(chain, tool_T,
                                            {"j1": math.pi / 2.0})
    # ee frame rotated with the joint: the +x tool offset points along +y.
    np.testing.assert_allclose(tip, [0.0, 0.6, 0.0], atol=1e-12)


def test_into_paper_is_positive_canvas_z():
    origin = np.zeros(3)
    R = np.eye(3)  # canvas z == base z, into the paper
    c = analyze_tracking_bag.project_to_canvas(origin, R, [0.0, 0.0, 0.002])
    np.testing.assert_allclose(c, [0.0, 0.0, 2.0], atol=1e-12)


def test_direction_classification():
    cls = analyze_tracking_bag.classify_direction
    up_y = [[0.0, i] for i in range(31)]
    assert cls(up_y) == "+Y"
    assert cls([[0.0, -i] for i in range(31)]) == "-Y"
    assert cls([[i, 0.0] for i in range(31)]) == "+X"
    assert cls([[-i, 0.0] for i in range(31)]) == "-X"
    assert cls([[i, i] for i in range(31)]) == "mixed"
    assert cls([[0.0, 0.0], [0.1, 0.1]]) == "static"


def test_segmentation_and_metrics_on_synthetic_bag(tmp_path):
    bag_dir, canvas, calib = _fixture_files(tmp_path)
    metrics, rates = analyze_tracking_bag.analyze(
        bag_dir, canvas, calib, plane_bias_mm=1.0)
    assert [m["segment"].label for m in metrics] == ["row_a", "row_b"]
    assert [m["segment"].type for m in metrics] == ["paint_stroke"] * 2

    row_a, row_b = metrics
    assert row_a["direction"] == "+Y"
    assert row_b["direction"] == "-X"
    # Injected +0.5 mm into-paper offset is recovered exactly, signed.
    np.testing.assert_allclose(row_a["normal_err_mean_mm"], 0.5, atol=1e-9)
    np.testing.assert_allclose(row_a["normal_err_min_mm"], 0.5, atol=1e-9)
    np.testing.assert_allclose(row_b["normal_err_mean_mm"], 0.0, atol=1e-9)
    # compression = plane_bias + actual canvas z
    np.testing.assert_allclose(row_a["compression_mean_mm"], 1.5, atol=1e-9)
    np.testing.assert_allclose(row_b["compression_mean_mm"], 1.0, atol=1e-9)
    assert row_a["tangential_err_max_mm"] < 1e-9
    # 51 samples over 0.5 s -> 100 Hz
    st = rates["/joint_trajectory_controller/controller_state"]
    assert 90.0 < st["rate_median_hz"] < 210.0
    assert rates["/joint_states"]["n"] == 20


def test_csv_export(tmp_path):
    bag_dir, canvas, calib = _fixture_files(tmp_path)
    metrics, _rates = analyze_tracking_bag.analyze(
        bag_dir, canvas, calib, plane_bias_mm=1.0)
    out = tmp_path / "out.csv"
    analyze_tracking_bag.write_csv(out, metrics)
    lines = out.read_text().strip().splitlines()
    assert lines[0].startswith("t_ns,command_index")
    assert len(lines) == 1 + sum(m["n_samples"] for m in metrics)


def test_canvas_math_matches_teach_canvas_calibration():
    # Wall fixture: A4 on a vertical wall in front of the arm.
    tl = [0.60, 0.105, 0.50]
    tr = [0.60, -0.105, 0.50]
    bl = [0.60, 0.105, 0.203]
    (origin, quat, width_m, height_m, _coeffs, _before,
     _after) = teach_canvas.compute_canvas_calibration(
        tl, tr, [0.60, 0.105, 0.203], [0.60, -0.105, 0.203],
        plane_bias_mm=0.0)
    R = analyze_tracking_bag.quat_to_matrix(*quat)
    p = analyze_tracking_bag.project_to_canvas
    np.testing.assert_allclose(p(np.asarray(origin), R, tl),
                               [0.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(p(np.asarray(origin), R, tr),
                               [width_m * 1000.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(p(np.asarray(origin), R, bl),
                               [0.0, height_m * 1000.0, 0.0], atol=1e-6)


def test_analysis_never_touches_the_ros_graph(tmp_path):
    bag_dir, canvas, calib = _fixture_files(tmp_path)
    analyze_tracking_bag.analyze(bag_dir, canvas, calib)
    import rclpy
    assert not rclpy.ok()  # never initialized, nothing published
    source = (_SCRIPTS / "analyze_tracking_bag.py").read_text()
    for forbidden in ("rclpy.init", "create_publisher", "create_node",
                      "rclpy.spin"):
        assert forbidden not in source
