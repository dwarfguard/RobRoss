import importlib.util
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "paint.launch.py"
SPEC = importlib.util.spec_from_file_location("paint_launch", LAUNCH_FILE)
paint_launch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(paint_launch)


def write_parameter_file(path, parameters):
    path.write_text(
        yaml.safe_dump(
            {"painting_executor": {"ros__parameters": parameters}},
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    return str(path)


def valid_calibration(dry_run_marker=None):
    parameters = {
        "ground_enabled": False,
        "canvas_backing_enabled": True,
        "tool_offset_xyz": [0.0, 0.0, 0.12],
    }
    if dry_run_marker is not None:
        parameters["dry_run"] = dry_run_marker
    return parameters


def test_require_file_rejects_missing_path(tmp_path):
    missing = tmp_path / "missing.yaml"

    with pytest.raises(RuntimeError, match="calibration_file is not a file"):
        paint_launch.require_file(str(missing), "calibration_file")


def test_calibration_rejects_pose_only_yaml(tmp_path):
    path = write_parameter_file(
        tmp_path / "canvas.yaml",
        {
            "canvas_origin_xyz": [0.5, 0.1, 0.2],
            "canvas_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
    )

    with pytest.raises(RuntimeError, match="ground_enabled"):
        paint_launch.validate_calibration_file(path)


@pytest.mark.parametrize("dry_run_marker", [None, True, False])
def test_calibration_does_not_enforce_dry_run(tmp_path, dry_run_marker):
    path = write_parameter_file(
        tmp_path / "calibration.yaml", valid_calibration(dry_run_marker)
    )

    paint_launch.validate_calibration_file(path)


def test_canvas_requires_taught_pose(tmp_path):
    path = write_parameter_file(
        tmp_path / "canvas.yaml", {"canvas_origin_xyz": [0.5, 0.1, 0.2]}
    )

    with pytest.raises(RuntimeError, match="canvas_quat_xyzw"):
        paint_launch.validate_canvas_file(path)


def test_robot_description_names_match():
    paint_launch.validate_robot_description_names(
        '<robot name="aubo_robot"/>', '<robot name="aubo_robot"/>'
    )


def test_robot_description_names_reject_mismatch():
    with pytest.raises(RuntimeError, match="does not match"):
        paint_launch.validate_robot_description_names(
            '<robot name="aubo_robot"/>', '<robot name="aubo_i5_robot"/>'
        )


def test_robot_description_names_reject_malformed_xml():
    with pytest.raises(RuntimeError, match="Cannot parse URDF"):
        paint_launch.validate_robot_description_names(
            '<robot name="aubo_robot">', '<robot name="aubo_robot"/>'
        )


def test_shipped_calibration_profiles_are_valid():
    for path in (PACKAGE_ROOT / "config").glob("*.yaml"):
        paint_launch.validate_calibration_file(str(path))


def test_shipped_hardware_profile_is_dry_run():
    path = PACKAGE_ROOT / "config" / "hardware_a4.yaml"
    parameters = yaml.safe_load(path.read_text(encoding="utf-8"))

    assert parameters["painting_executor"]["ros__parameters"]["dry_run"] is True


def test_shipped_profiles_use_controller_period_interpolation():
    # Remediation plan Phase 1: Cartesian trajectories are resampled at the
    # controller period and validated with a dedicated canvas-normal limit;
    # the old totg_resample_dt no longer exists.
    for name in (
        "hardware_a4.yaml",
        "demo_v1_rviz.yaml",
        "rviz_taught_a4.yaml",
        "rviz_wall_a4.yaml",
    ):
        path = PACKAGE_ROOT / "config" / name
        params = yaml.safe_load(path.read_text(encoding="utf-8"))
        params = params["painting_executor"]["ros__parameters"]
        assert params["controller_sample_dt"] == 0.005, name
        assert params["max_cartesian_normal_deviation_mm"] == 0.2, name
        assert "totg_resample_dt" not in params, name
