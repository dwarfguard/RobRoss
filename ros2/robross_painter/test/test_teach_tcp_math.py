"""Unit tests for the pure calibration math in scripts/teach_tcp.py."""

import importlib.util
import math
from pathlib import Path

import numpy as np

_MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "teach_tcp.py"
)
_spec = importlib.util.spec_from_file_location("teach_tcp", _MODULE_PATH)
teach_tcp = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(teach_tcp)


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rpy_to_matrix(roll, pitch, yaw):
    # Matches tf2 setRPY: R = Rz(yaw) Ry(pitch) Rx(roll).
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


def _poses_touching(world_point, tool_point, rpys):
    """ee poses (p, R) whose tool_point all contacts the same world_point."""
    poses = []
    for roll, pitch, yaw in rpys:
        r = _rpy_to_matrix(roll, pitch, yaw)
        p = np.asarray(world_point, dtype=float) - r @ np.asarray(tool_point)
        poses.append((p, r))
    return poses


# --- solve_pivot ------------------------------------------------------------


def test_pivot_recovers_known_tool_point():
    world = [0.55, 0.10, 0.15]
    tool = [0.001208, -0.06034, 0.090753]
    rpys = [
        (0.2, 0.1, 0.0),
        (-0.3, 0.25, 0.5),
        (0.15, -0.4, -0.3),
        (-0.1, 0.35, 0.6),
        (0.4, -0.2, 0.2),
    ]
    poses = _poses_touching(world, tool, rpys)
    t, p, rms_mm, rank, cond = teach_tcp.solve_pivot(poses)
    np.testing.assert_allclose(t, tool, atol=1e-6)
    np.testing.assert_allclose(p, world, atol=1e-6)
    assert rms_mm < 1e-3           # < 1 micron of scatter on perfect data
    assert rank == 6
    assert cond < 1e3              # varied orientations => well-conditioned


def test_pivot_reports_scatter_on_noisy_touches():
    world = [0.55, 0.10, 0.15]
    tool = [0.001208, -0.06034, 0.090753]
    rpys = [
        (0.2, 0.1, 0.0),
        (-0.3, 0.25, 0.5),
        (0.15, -0.4, -0.3),
        (-0.1, 0.35, 0.6),
    ]
    poses = _poses_touching(world, tool, rpys)
    rng = np.random.default_rng(0)
    noisy = [(p + rng.normal(0.0, 0.0003, 3), r) for p, r in poses]  # 0.3 mm
    _t, _p, rms_mm, _rank, _cond = teach_tcp.solve_pivot(noisy)
    assert 0.05 < rms_mm < 2.0     # scatter reported in a sane mm range


def test_pivot_flags_identical_orientations_as_rank_deficient():
    world = [0.55, 0.10, 0.15]
    tool = [0.001208, -0.06034, 0.090753]
    # Operator never reoriented the wrist: every touch is the same pose, so the
    # stacked system cannot pin down the tool point (rank 3, not 6).
    rpys = [(0.10, 0.10, 0.10)] * 4
    poses = _poses_touching(world, tool, rpys)
    _t, _p, _rms, rank, _cond = teach_tcp.solve_pivot(poses)
    assert rank < 6


def test_pivot_condition_number_grows_as_orientation_spread_shrinks():
    world = [0.55, 0.10, 0.15]
    tool = [0.001208, -0.06034, 0.090753]

    def cond_for(scale):
        rpys = [
            (0.0, 0.0, 0.0),
            (scale, 0.5 * scale, -0.3 * scale),
            (-0.4 * scale, scale, 0.2 * scale),
            (0.3 * scale, -0.2 * scale, scale),
        ]
        return teach_tcp.solve_pivot(_poses_touching(world, tool, rpys))[4]

    # A wide spread is well-conditioned (well below the 100 cond_warn default);
    # a tiny spread blows the condition number up past it.
    assert cond_for(1.0) < 20.0                        # ~57 deg spread
    assert cond_for(math.radians(0.1)) > 100.0         # ~0.1 deg spread


# --- axis / rpy -------------------------------------------------------------


def test_rpy_from_axis_identity_for_z():
    np.testing.assert_allclose(
        teach_tcp.rpy_from_axis([0.0, 0.0, 1.0]), [0.0, 0.0, 0.0], atol=1e-9
    )


def test_rpy_from_axis_round_trips_through_setrpy():
    for axis in (
        [0.0, 0.0, 1.0],
        [0.1, 0.0, 1.0],
        [0.0, -0.2, 1.0],
        [0.3, 0.2, 0.9],
        [-0.4, 0.1, 0.8],
    ):
        v = np.asarray(axis, dtype=float)
        v /= np.linalg.norm(v)
        roll, pitch, yaw = teach_tcp.rpy_from_axis(v)
        # Rebuilding the tool frame and applying it to ee +Z gives axis back.
        recovered = _rpy_to_matrix(roll, pitch, yaw) @ np.array([0.0, 0.0, 1.0])
        np.testing.assert_allclose(recovered, v, atol=1e-9)


def test_minimal_rotation_has_no_roll_about_axis():
    # The minimal rotation ee +Z -> v turns about k = z x v, which is
    # perpendicular to v (no roll about the pen axis) and is itself fixed by R.
    v = np.array([0.3, 0.2, 0.9])
    v /= np.linalg.norm(v)
    r = teach_tcp.minimal_rotation_z_to(v)
    np.testing.assert_allclose(r.T @ r, np.eye(3), atol=1e-9)  # valid rotation
    k = np.cross([0.0, 0.0, 1.0], v)
    k /= np.linalg.norm(k)
    assert abs(float(np.dot(k, v))) < 1e-9          # rotation axis _|_ pen axis
    np.testing.assert_allclose(r @ k, k, atol=1e-9)  # rotation axis is fixed
    np.testing.assert_allclose(r @ np.array([0.0, 0.0, 1.0]), v, atol=1e-9)
