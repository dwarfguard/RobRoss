import importlib.util
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "cartesian_path_probe.launch.py"
SPEC = importlib.util.spec_from_file_location("cartesian_probe_launch", LAUNCH_FILE)
probe_launch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe_launch)


def test_require_file_rejects_missing_artifact(tmp_path):
    with pytest.raises(RuntimeError, match="recorded_request_file is not a file"):
        probe_launch.require_file(str(tmp_path / "missing.json"), "recorded_request_file")


@pytest.mark.parametrize("value", ["0", "-1", "not-an-integer"])
def test_repetitions_must_be_positive(value):
    with pytest.raises(RuntimeError, match="must be a positive integer"):
        probe_launch.positive_integer(value, "repetitions")


def test_probe_requires_isolated_move_group_confirmation():
    with pytest.raises(RuntimeError, match="must be true"):
        probe_launch.required_confirmation("false")
    assert probe_launch.required_confirmation("true") is True


def test_probe_source_has_no_execution_interface():
    source = (PACKAGE_ROOT / "src" / "cartesian_path_probe.cpp").read_text(
        encoding="utf-8"
    )
    forbidden = (
        "move_group_interface",
        "ExecuteTrajectory",
        "asyncExecute",
        ".execute(",
        ".move(",
        "/execute_trajectory",
    )
    for token in forbidden:
        assert token not in source


def test_executor_retreats_before_running_failure_probes():
    source = (PACKAGE_ROOT / "src" / "painting_executor.cpp").read_text(
        encoding="utf-8"
    )
    failure_log = source.index('"Command %d (\'%s\', label \'%s\') failed, aborting"')
    retreat = source.index("attemptRetreat()", failure_log)
    diagnostics = source.index("processPendingCartesianFailures(retreat)", failure_log)
    assert retreat < diagnostics
    assert "moveCartesian({ hover }, MotionKind::Lifting, false)" in source


def test_executor_diagnoses_failed_pen_up_travel_before_fallback():
    source = (PACKAGE_ROOT / "src" / "painting_executor.cpp").read_text(
        encoding="utf-8"
    )
    travel = source.index("bool doMoveTo(double x_mm, double y_mm)")
    cartesian = source.index("ok = moveCartesian({ target })", travel)
    diagnostics = source.index(
        "processPendingCartesianFailures(RetreatOutcome::NotNeeded)", cartesian
    )
    fallback = source.index("ok = moveJointSpace(target)", diagnostics)
    assert cartesian < diagnostics < fallback
