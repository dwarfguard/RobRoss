import importlib.util
import json
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


def test_require_directory_accepts_empty_and_rejects_file(tmp_path):
    assert paint_launch.require_directory("", "artifact_dir") == ""
    file_path = tmp_path / "not_a_directory"
    file_path.write_text("x", encoding="utf-8")
    with pytest.raises(RuntimeError, match="is not a directory"):
        paint_launch.require_directory(str(file_path), "artifact_dir")


@pytest.mark.parametrize("value", ["-1", "bad"])
def test_capture_command_index_must_be_nonnegative(value):
    with pytest.raises(RuntimeError, match="must be a non-negative integer"):
        paint_launch.nonnegative_integer(value, "capture_command")


@pytest.mark.parametrize("value", ["-0.1", "nan", "bad"])
def test_canvas_dimension_tolerance_must_be_nonnegative(value):
    with pytest.raises(RuntimeError, match="must be a non-negative number"):
        paint_launch.nonnegative_number(value, "canvas_dimension_tolerance_mm")


def test_load_path_canvas_dimensions_accepts_4k_landscape(tmp_path):
    path = tmp_path / "paths.json"
    path.write_text(
        json.dumps({"canvas": {"width_mm": 525.0, "height_mm": 375.0}}),
        encoding="utf-8",
    )
    assert paint_launch.load_path_canvas_dimensions(path) == (525.0, 375.0)


def test_canvas_dimension_warning_only_exceeds_tolerance():
    assert paint_launch.canvas_dimension_warnings(
        (525.0, 375.0),
        {"taught_canvas_width_mm": 523.0, "taught_canvas_height_mm": 379.0},
        5.0,
    ) == []
    warnings = paint_launch.canvas_dimension_warnings(
        (525.0, 375.0),
        {"taught_canvas_width_mm": 518.0, "taught_canvas_height_mm": 381.0},
        5.0,
    )
    assert len(warnings) == 2
    assert "width" in warnings[0]
    assert "height" in warnings[1]


def test_canvas_dimension_warning_supports_legacy_calibration():
    assert paint_launch.canvas_dimension_warnings((525.0, 375.0), {}, 5.0) == []


@pytest.mark.parametrize(
    ("value", "expected"),
    [("true", True), ("ON", True), ("0", False), ("no", False)],
)
def test_boolean_value(value, expected):
    assert paint_launch.boolean_value(value, "flag") is expected


def test_boolean_value_rejects_unknown_value():
    with pytest.raises(RuntimeError, match="must be a boolean"):
        paint_launch.boolean_value("sometimes", "flag")


def identity(package, **overrides):
    contents = {
        "schema_version": 1,
        "package": package,
        "available": True,
        "source_clean": True,
        "repository": f"https://example.com/{package}.git",
        "commit": "a" * 40,
        "reason": "",
    }
    contents.update(overrides)
    return json.dumps(contents)


def install_identity_mocks(monkeypatch, contents, identity_prefix="/overlay"):
    monkeypatch.setattr(
        paint_launch,
        "get_resource",
        lambda resource_type, package: (contents, identity_prefix),
    )
    monkeypatch.setattr(
        paint_launch, "get_package_prefix", lambda package: "/overlay"
    )


def test_build_identity_accepts_exact_clean_source_build(monkeypatch):
    package = "aubo_ros2_driver"
    install_identity_mocks(monkeypatch, identity(package))

    result = paint_launch.validate_build_identity(
        package, f"https://example.com/{package}.git", "a" * 40
    )
    assert result["commit"] == "a" * 40


def test_build_identity_rejects_missing_resource(monkeypatch):
    def missing_resource(resource_type, package):
        raise LookupError(package)

    monkeypatch.setattr(paint_launch, "get_resource", missing_resource)
    with pytest.raises(RuntimeError, match="source build identity is unavailable"):
        paint_launch.validate_build_identity(
            "joint_trajectory_controller",
            "https://example.com/ros2_controllers.git",
            "a" * 40,
        )


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"available": False, "reason": "dirty source"}, "dirty source"),
        ({"source_clean": False}, "not a qualified source build"),
        ({"repository": "https://example.com/other.git"}, "repository mismatch"),
        ({"commit": "b" * 40}, "commit mismatch"),
    ],
)
def test_build_identity_rejects_unqualified_or_mismatched_build(
    monkeypatch, overrides, message
):
    package = "aubo_ros2_driver"
    install_identity_mocks(monkeypatch, identity(package, **overrides))

    with pytest.raises(RuntimeError, match=message):
        paint_launch.validate_build_identity(
            package, f"https://example.com/{package}.git", "a" * 40
        )


def test_build_identity_rejects_marker_from_shadowed_prefix(monkeypatch):
    package = "joint_trajectory_controller"
    install_identity_mocks(monkeypatch, identity(package), "/underlay")

    with pytest.raises(RuntimeError, match="does not describe the active package"):
        paint_launch.validate_build_identity(
            package, f"https://example.com/{package}.git", "a" * 40
        )


def test_qualification_requires_controller_pin_before_resource_lookup(monkeypatch):
    calls = []

    def validate(package, repository, commit):
        calls.append((package, repository, commit))
        if not repository:
            raise RuntimeError(f"expected repository is required for {package}")
        return {"package": package, "repository": repository, "commit": commit}

    monkeypatch.setattr(paint_launch, "validate_build_identity", validate)

    with pytest.raises(RuntimeError, match="joint_trajectory_controller"):
        paint_launch.validate_qualified_build(
            "https://example.com/driver.git", "a" * 40, "", ""
        )
    assert calls[0][0] == "aubo_ros2_driver"


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


def qualified_parameters():
    parameters = valid_calibration(False)
    parameters["controller_sample_dt"] = 0.008
    for name in paint_launch.QUALIFIED_CHECKS:
        parameters[name] = True
    return parameters


def test_qualification_parameters_accept_frozen_timing_and_all_checks():
    paint_launch.validate_qualification_parameters(
        qualified_parameters(), 125, 0.008, 200, 0.05
    )


def test_qualification_parameters_reject_disabled_check():
    parameters = qualified_parameters()
    parameters["endpoint_settle_enabled"] = False

    with pytest.raises(RuntimeError, match="endpoint_settle_enabled"):
        paint_launch.validate_qualification_parameters(
            parameters, 125, 0.008, 200, 0.05
        )


def test_qualification_parameters_reject_unqualified_timing_pair():
    with pytest.raises(RuntimeError, match="125 Hz"):
        paint_launch.validate_qualification_parameters(
            qualified_parameters(), 200, 0.005, 200, 0.05
        )


def test_provenance_manifest_is_structured_and_atomic(tmp_path, monkeypatch):
    monkeypatch.setattr(
        paint_launch,
        "get_package_prefix",
        lambda package: str(tmp_path / "missing-prefix"),
    )
    monkeypatch.setattr(
        paint_launch,
        "git_identity",
        lambda path: {"path": str(path), "revision": "a" * 40, "dirty": False},
    )
    input_path = tmp_path / "paths.json"
    input_path.write_text("{}", encoding="utf-8")

    destination = paint_launch.write_provenance_manifest(
        tmp_path,
        "session-a",
        "aubo_i5_calibrated",
        {"paths_file": str(input_path)},
        qualified_parameters(),
        '<robot name="aubo"><joint name="j1" type="revolute"/></robot>',
        '<robot name="aubo"><group name="manipulator"><joint name="j1"/></group></robot>',
        {"controller_update_rate_hz": 125},
        {"status": "pass"},
        {"status": "running"},
        {
            "aubo_ros2_driver": {"commit": "a" * 40},
            "joint_trajectory_controller": {"commit": "b" * 40},
        },
    )

    manifest = json.loads(Path(destination).read_text(encoding="utf-8"))
    assert manifest["session_id"] == "session-a"
    assert all(manifest["coverage"].values())
    assert (
        manifest["qualified_build_identities"]["joint_trajectory_controller"][
            "commit"
        ]
        == "b" * 40
    )
    assert manifest["robot"]["urdf_joint_order"] == ["j1"]
    assert not list(tmp_path.glob("*.tmp"))


def test_provenance_manifest_marks_unqualified_build_as_engineering(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(
        paint_launch,
        "get_package_prefix",
        lambda package: str(tmp_path / "missing-prefix"),
    )
    monkeypatch.setattr(
        paint_launch,
        "git_identity",
        lambda path: {"path": str(path), "revision": "a" * 40, "dirty": True},
    )
    input_path = tmp_path / "paths.json"
    input_path.write_text("{}", encoding="utf-8")

    destination = paint_launch.write_provenance_manifest(
        tmp_path,
        "session-engineering",
        "aubo_i5_calibrated",
        {"paths_file": str(input_path)},
        qualified_parameters(),
        '<robot name="aubo"><joint name="j1" type="revolute"/></robot>',
        '<robot name="aubo"><group name="manipulator"><joint name="j1"/></group></robot>',
        {"controller_update_rate_hz": 125},
        {"status": "pass"},
        {"status": "running"},
        None,
    )

    manifest = json.loads(Path(destination).read_text(encoding="utf-8"))
    assert manifest["status"] == "engineering"
    assert manifest["qualified_build_identities"] is None
    assert manifest["coverage"]["qualified_build_identity"] is False


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


def test_shipped_hardware_profile_is_safe_template():
    path = PACKAGE_ROOT / "config" / "hardware_a4.yaml"
    parameters = yaml.safe_load(path.read_text(encoding="utf-8"))

    parameters = parameters["painting_executor"]["ros__parameters"]
    assert parameters["dry_run"] is True
    assert all(parameters[name] is True for name in paint_launch.QUALIFIED_CHECKS)


def test_shipped_4k_landscape_profile_is_safe_template():
    path = PACKAGE_ROOT / "config" / "hardware_4k_landscape.yaml"
    parameters = yaml.safe_load(path.read_text(encoding="utf-8"))
    parameters = parameters["painting_executor"]["ros__parameters"]

    assert parameters["dry_run"] is True
    assert parameters["canvas_backing_size_xy_m"] == [0.0, 0.0]
    assert all(parameters[name] is True for name in paint_launch.QUALIFIED_CHECKS)


def test_hardware_representative_profiles_share_tool_geometry():
    geometry_keys = (
        "tool_offset_xyz",
        "tool_offset_rpy",
        "tool_spin_deg",
        "claw_collision_size_xyz",
        "claw_collision_offset_xyz",
    )
    profiles = {}
    for name in (
        "hardware_a4.yaml",
        "hardware_4k_landscape.yaml",
        "rviz_wall_a4.yaml",
        "rviz_taught_a4.yaml",
    ):
        path = PACKAGE_ROOT / "config" / name
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        profiles[name] = data["painting_executor"]["ros__parameters"]

    hardware = profiles["hardware_a4.yaml"]
    for name, parameters in profiles.items():
        assert {key: parameters[key] for key in geometry_keys} == {
            key: hardware[key] for key in geometry_keys
        }, name


def test_shipped_profiles_use_controller_period_interpolation():
    # Cartesian trajectories are resampled at the controller period and
    # validated with a dedicated canvas-normal limit; the old
    # totg_resample_dt no longer exists.
    sample_periods = {
        "hardware_a4.yaml": 0.008,
        "hardware_4k_landscape.yaml": 0.008,
        "demo_v1_rviz.yaml": 0.008,
        "rviz_taught_a4.yaml": 0.008,
        "rviz_wall_a4.yaml": 0.008,
    }
    check_keys = (
        "cartesian_deviation_check_enabled",
        "cartesian_normal_deviation_check_enabled",
        "cartesian_orientation_deviation_check_enabled",
        "execution_joint_error_check_enabled",
        "execution_tip_error_check_enabled",
        "execution_tip_orientation_error_check_enabled",
        "endpoint_settle_enabled",
    )
    for name, sample_period in sample_periods.items():
        path = PACKAGE_ROOT / "config" / name
        params = yaml.safe_load(path.read_text(encoding="utf-8"))
        params = params["painting_executor"]["ros__parameters"]
        assert params["controller_sample_dt"] == sample_period, name
        assert {key: params[key] for key in check_keys} == {
            key: True for key in check_keys
        }, name
        assert params["max_cartesian_normal_deviation_mm"] > 0.0, name
        assert params["max_execution_joint_error_deg"] == 2.0, name
        assert "totg_resample_dt" not in params, name


def test_canvas_file_accepts_measured_dimensions(tmp_path):
    path = write_parameter_file(
        tmp_path / "canvas.yaml",
        {
            "canvas_origin_xyz": [0.4, 0.2, 0.6],
            "canvas_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
            "taught_canvas_width_mm": 525.0,
            "taught_canvas_height_mm": 375.0,
        },
    )
    paint_launch.validate_canvas_file(path)


def test_canvas_file_rejects_partial_measured_dimensions(tmp_path):
    path = write_parameter_file(
        tmp_path / "canvas.yaml",
        {
            "canvas_origin_xyz": [0.4, 0.2, 0.6],
            "canvas_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
            "taught_canvas_width_mm": 525.0,
        },
    )
    with pytest.raises(RuntimeError, match="both taught canvas dimensions"):
        paint_launch.validate_canvas_file(path)
