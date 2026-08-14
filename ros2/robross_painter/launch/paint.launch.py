import datetime as dt
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from ament_index_python.resources import get_resource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


QUALIFIED_CHECKS = (
    "cartesian_deviation_check_enabled",
    "cartesian_normal_deviation_check_enabled",
    "cartesian_orientation_deviation_check_enabled",
    "execution_joint_error_check_enabled",
    "execution_tip_error_check_enabled",
    "execution_tip_orientation_error_check_enabled",
    "endpoint_settle_enabled",
)


def load_host_module(name):
    candidates = [
        Path(__file__).resolve().parents[1] / "scripts" / f"{name}.py",
        Path(get_package_prefix("robross_painter"))
        / "lib"
        / "robross_painter"
        / f"{name}.py",
    ]
    for path in candidates:
        if path.is_file():
            scripts_dir = str(path.parent)
            if scripts_dir not in sys.path:
                sys.path.insert(0, scripts_dir)
            spec = importlib.util.spec_from_file_location(name, path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    raise RuntimeError(f"Cannot locate installed {name}.py")


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def require_file(file_path, label):
    resolved_path = os.path.abspath(os.path.expanduser(file_path))
    if not os.path.isfile(resolved_path):
        raise RuntimeError(f"{label} is not a file: {resolved_path}")
    return resolved_path


def require_directory(directory_path, label):
    if not directory_path:
        return ""
    resolved_path = os.path.abspath(os.path.expanduser(directory_path))
    if not os.path.isdir(resolved_path):
        raise RuntimeError(f"{label} is not a directory: {resolved_path}")
    if not os.access(resolved_path, os.W_OK):
        raise RuntimeError(f"{label} is not writable: {resolved_path}")
    return resolved_path


def nonnegative_integer(value, label):
    try:
        parsed = int(value)
    except ValueError as error:
        raise RuntimeError(f"{label} must be a non-negative integer") from error
    if parsed < 0:
        raise RuntimeError(f"{label} must be a non-negative integer")
    return parsed


def boolean_value(value, label):
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise RuntimeError(f"{label} must be a boolean")


def validate_build_identity(package_name, expected_repository, expected_commit):
    if not expected_repository:
        raise RuntimeError(f"expected repository is required for {package_name}")
    if not re.fullmatch(r"[0-9a-f]{40}", expected_commit):
        raise RuntimeError(
            f"expected commit for {package_name} must be a full lowercase Git SHA"
        )
    try:
        contents, identity_prefix = get_resource(
            "robross_build_identity", package_name
        )
    except (LookupError, OSError) as error:
        raise RuntimeError(
            f"qualified source build identity is unavailable for {package_name}"
        ) from error
    try:
        identity = json.loads(contents)
    except (TypeError, json.JSONDecodeError) as error:
        raise RuntimeError(
            f"build identity for {package_name} is not valid JSON"
        ) from error

    package_prefix = get_package_prefix(package_name)
    if os.path.realpath(identity_prefix) != os.path.realpath(package_prefix):
        raise RuntimeError(
            f"build identity for {package_name} does not describe the active "
            f"package prefix {package_prefix}"
        )
    if identity.get("schema_version") != 1 or identity.get("package") != package_name:
        raise RuntimeError(f"build identity for {package_name} has invalid metadata")
    if identity.get("available") is not True or identity.get("source_clean") is not True:
        reason = identity.get("reason") or "source identity was not available at build"
        raise RuntimeError(f"{package_name} is not a qualified source build: {reason}")
    if identity.get("repository") != expected_repository:
        raise RuntimeError(f"repository mismatch for {package_name}")
    if identity.get("commit") != expected_commit:
        raise RuntimeError(f"commit mismatch for {package_name}")
    return identity


def validate_qualified_build(
    driver_repository,
    driver_commit,
    controller_repository,
    controller_commit,
):
    driver_identity = validate_build_identity(
        "aubo_ros2_driver", driver_repository, driver_commit
    )
    controller_identity = validate_build_identity(
        "joint_trajectory_controller",
        controller_repository,
        controller_commit,
    )
    return {
        "aubo_ros2_driver": driver_identity,
        "joint_trajectory_controller": controller_identity,
    }


def load_parameter_file(file_path, label):
    try:
        with open(file_path, "r", encoding="utf-8") as file:
            contents = yaml.safe_load(file)
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(f"Cannot load {label} '{file_path}': {error}") from error

    try:
        parameters = contents["painting_executor"]["ros__parameters"]
    except (KeyError, TypeError):
        raise RuntimeError(
            f"{label} '{file_path}' must contain "
            "painting_executor.ros__parameters"
        ) from None
    if not isinstance(parameters, dict):
        raise RuntimeError(
            f"{label} '{file_path}' painting_executor.ros__parameters "
            "must be a mapping"
        )
    return parameters


def validate_calibration_file(file_path):
    parameters = load_parameter_file(file_path, "calibration_file")
    required = {
        "ground_enabled",
        "canvas_backing_enabled",
        "tool_offset_xyz",
    }
    missing = sorted(required.difference(parameters))
    if missing:
        raise RuntimeError(
            f"calibration_file '{file_path}' is missing required parameter(s): "
            + ", ".join(missing)
        )


def validate_canvas_file(file_path):
    parameters = load_parameter_file(file_path, "canvas_file")
    required = {"canvas_origin_xyz", "canvas_quat_xyzw"}
    missing = sorted(required.difference(parameters))
    if missing:
        raise RuntimeError(
            f"canvas_file '{file_path}' is missing required parameter(s): "
            + ", ".join(missing)
        )


def validate_qualification_parameters(
    parameters,
    controller_rate_hz,
    servoj_time,
    rtde_state_rate,
    rtde_state_max_age,
):
    disabled = [name for name in QUALIFIED_CHECKS if parameters.get(name) is not True]
    if disabled:
        raise RuntimeError(
            "qualification_run requires enabled conformance checks: "
            + ", ".join(disabled)
        )
    if parameters.get("controller_sample_dt") != 0.008:
        raise RuntimeError("qualification_run requires controller_sample_dt=0.008")
    expected = (125, 0.008, 200, 0.05)
    actual = (
        int(controller_rate_hz),
        float(servoj_time),
        int(rtde_state_rate),
        float(rtde_state_max_age),
    )
    if actual != expected:
        raise RuntimeError(
            "qualification_run requires controller/ServoJ/RTDE tuple "
            f"125 Hz, 0.008 s, 200 Hz, 0.05 s; got {actual}"
        )


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_identity(path):
    identity = {"path": str(path), "revision": None, "dirty": None}
    try:
        revision = subprocess.run(
            ["git", "-C", str(path), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        )
        status = subprocess.run(
            ["git", "-C", str(path), "status", "--porcelain"],
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        )
        identity.update(revision=revision.stdout.strip(), dirty=bool(status.stdout))
    except (OSError, subprocess.SubprocessError):
        pass
    return identity


def executable_identity(path):
    identity = {"path": str(path), "sha256": None, "elf_build_id": None}
    if not path.is_file():
        return identity
    identity["sha256"] = sha256_file(path)
    try:
        result = subprocess.run(
            ["readelf", "-n", str(path)],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
        for line in result.stdout.splitlines():
            if "Build ID:" in line:
                identity["elf_build_id"] = line.split("Build ID:", 1)[1].strip()
                break
    except (OSError, subprocess.SubprocessError):
        pass
    return identity


def write_provenance_manifest(
    directory,
    session_id,
    aubo_type,
    inputs,
    effective_parameters,
    urdf_xml,
    srdf_xml,
    timing,
    preflight_receipt,
    clock_monitor,
    qualified_build_identities,
):
    host_time = load_host_module("host_time_preflight")
    repo_root = Path(
        os.environ.get("ROBROSS_REPO", Path(__file__).resolve().parents[3])
    ).expanduser().resolve()
    workspace_src = repo_root.parent
    executable = (
        Path(get_package_prefix("robross_painter"))
        / "lib"
        / "robross_painter"
        / "painting_executor"
    )
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    destination = Path(directory) / f"painting_startup_{timestamp}_{os.getpid()}.json"
    urdf_root = ET.fromstring(urdf_xml)
    srdf_root = ET.fromstring(srdf_xml)
    manifest = {
        "schema_version": 1,
        "kind": "robross_painting_startup_provenance",
        "status": "qualified",
        "session_id": session_id,
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat().replace(
            "+00:00", "Z"
        ),
        "host": socket.gethostname(),
        "pid": os.getpid(),
        "coverage": {
            "session_identity": True,
            "host_time_preflight": True,
            "host_clock_monitor": True,
            "source_revisions": True,
            "executable_identity": True,
            "input_hashes": True,
            "effective_parameters": True,
            "robot_descriptions": True,
            "joint_order": True,
            "timing_tuple": True,
        },
        "host_time_preflight": preflight_receipt,
        "host_clock_monitor": clock_monitor,
        "sources": {
            "robross": git_identity(repo_root),
            "aubo_ros2_driver": git_identity(workspace_src / "aubo_ros2_driver"),
            "aubo_description": git_identity(
                workspace_src / "aubo_ros2_driver" / "aubo_description"
            ),
        },
        "qualified_build_identities": qualified_build_identities,
        "executable": executable_identity(executable),
        "inputs": {
            label: {"path": path, "sha256": sha256_file(path)}
            for label, path in inputs.items()
            if path
        },
        "robot": {
            "aubo_type": aubo_type,
            "urdf_sha256": hashlib.sha256(urdf_xml.encode()).hexdigest(),
            "srdf_sha256": hashlib.sha256(srdf_xml.encode()).hexdigest(),
            "urdf_joint_order": [
                joint.get("name")
                for joint in urdf_root.findall("joint")
                if joint.get("type") != "fixed"
            ],
            "srdf_group_joint_order": [
                joint.get("name")
                for group in srdf_root.findall("group")
                for joint in group.findall("joint")
            ],
        },
        "timing": timing,
        "effective_parameters": effective_parameters,
    }
    host_time.atomic_write_json(destination, manifest)
    return str(destination)


def validate_robot_description_names(urdf_xml, srdf_xml):
    names = []
    for label, description in (("URDF", urdf_xml), ("SRDF", srdf_xml)):
        try:
            root = ET.fromstring(description)
        except ET.ParseError as error:
            raise RuntimeError(f"Cannot parse {label}: {error}") from error
        if root.tag != "robot" or not root.get("name"):
            raise RuntimeError(f"{label} must have a named <robot> root")
        names.append(root.get("name"))

    if names[0] != names[1]:
        raise RuntimeError(
            f"URDF robot name '{names[0]}' does not match "
            f"SRDF robot name '{names[1]}'"
        )


def launch_setup(context, *args, **kwargs):
    aubo_type = LaunchConfiguration("aubo_type")
    paths_file = LaunchConfiguration("paths_file")
    calibration_file = LaunchConfiguration("calibration_file")
    canvas_file = LaunchConfiguration("canvas_file")
    artifact_dir = LaunchConfiguration("cartesian_failure_artifact_dir")
    capture_command = LaunchConfiguration("cartesian_capture_command_index")
    qualification_run = boolean_value(
        LaunchConfiguration("qualification_run").perform(context),
        "qualification_run",
    )
    session_id = LaunchConfiguration("session_id").perform(context)
    provenance_dir = LaunchConfiguration("provenance_dir").perform(context)
    preflight_receipt_path = LaunchConfiguration("preflight_receipt").perform(context)
    clock_monitor_path = LaunchConfiguration("clock_monitor_file").perform(context)

    require_qualified_build = boolean_value(
        LaunchConfiguration("require_qualified_build").perform(context),
        "require_qualified_build",
    )
    if qualification_run and not require_qualified_build:
        raise RuntimeError("qualification_run requires require_qualified_build=true")
    qualified_build_identities = None
    if require_qualified_build:
        qualified_build_identities = validate_qualified_build(
            LaunchConfiguration("expected_driver_repository").perform(context),
            LaunchConfiguration("expected_driver_commit").perform(context),
            LaunchConfiguration("expected_controller_repository").perform(context),
            LaunchConfiguration("expected_controller_commit").perform(context),
        )

    # Same robot_description the driver/moveit launches build, so the
    # MoveGroupInterface in the executor sees an identical model.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("aubo_description"), "urdf/xacro/inc/aubo_ros2.xacro"]
            ),
            " ",
            "aubo_type:=",
            aubo_type,
            " ",
        ]
    )
    robot_description_xml = robot_description_content.perform(context)
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("aubo_moveit_config"), "config", "aubo_robot.srdf"]
            ),
        ]
    )
    robot_description_semantic_xml = robot_description_semantic_content.perform(context)
    validate_robot_description_names(
        robot_description_xml, robot_description_semantic_xml
    )
    robot_description = {"robot_description": robot_description_xml}
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_xml
    }
    kinematics_yaml = load_yaml("aubo_moveit_config", "config/kinematics.yaml")

    calibration_file_path = require_file(
        calibration_file.perform(context), "calibration_file"
    )
    validate_calibration_file(calibration_file_path)
    calibration_parameters = load_parameter_file(
        calibration_file_path, "calibration_file"
    )
    paths_file_path = require_file(paths_file.perform(context), "paths_file")

    parameters = [
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        calibration_file_path,
    ]
    # Taught canvas pose (teach_canvas.py output) layered after the base
    # calibration so its canvas_origin_xyz / canvas_quat_xyzw win.
    canvas_file_path = canvas_file.perform(context)
    if canvas_file_path:
        canvas_file_path = require_file(canvas_file_path, "canvas_file")
        validate_canvas_file(canvas_file_path)
        parameters.append(canvas_file_path)
    artifact_dir_path = require_directory(
        artifact_dir.perform(context), "cartesian_failure_artifact_dir"
    )
    parameters.append(
        {
            "paths_file": paths_file_path,
            "cartesian_failure_artifact_dir": artifact_dir_path,
            "cartesian_capture_command_index": nonnegative_integer(
                capture_command.perform(context),
                "cartesian_capture_command_index",
            ),
        }
    )

    if qualification_run:
        if not session_id:
            raise RuntimeError("qualification_run requires a non-empty session_id")
        if not canvas_file_path:
            raise RuntimeError("qualification_run requires an explicit canvas_file")
        provenance_dir_path = require_directory(provenance_dir, "provenance_dir")
        if not provenance_dir_path:
            raise RuntimeError("qualification_run requires provenance_dir")
        preflight_receipt_path = require_file(
            preflight_receipt_path, "preflight_receipt"
        )
        clock_monitor_path = require_file(clock_monitor_path, "clock_monitor_file")
        host_time = load_host_module("host_time_preflight")
        host_monitor = load_host_module("host_clock_monitor")
        receipt_errors, preflight_receipt = host_time.validate_receipt(
            preflight_receipt_path, session_id, require_controller_fifo=True
        )
        monitor_errors, clock_monitor = host_monitor.validate_monitor(
            clock_monitor_path, session_id
        )
        if receipt_errors:
            raise RuntimeError(
                "preflight_receipt rejected: " + "; ".join(receipt_errors)
            )
        if monitor_errors:
            raise RuntimeError(
                "clock_monitor_file rejected: " + "; ".join(monitor_errors)
            )
        timing = {
            "controller_update_rate_hz": int(
                LaunchConfiguration("controller_update_rate_hz").perform(context)
            ),
            "servoj_time_s": float(
                LaunchConfiguration("servoj_time").perform(context)
            ),
            "rtde_state_rate_hz": int(
                LaunchConfiguration("rtde_state_rate").perform(context)
            ),
            "rtde_state_max_age_s": float(
                LaunchConfiguration("rtde_state_max_age").perform(context)
            ),
        }
        validate_qualification_parameters(
            calibration_parameters,
            timing["controller_update_rate_hz"],
            timing["servoj_time_s"],
            timing["rtde_state_rate_hz"],
            timing["rtde_state_max_age_s"],
        )
        effective_parameters = dict(calibration_parameters)
        effective_parameters.update(
            load_parameter_file(canvas_file_path, "canvas_file")
        )
        effective_parameters.update(parameters[-1])
        manifest = write_provenance_manifest(
            provenance_dir_path,
            session_id,
            aubo_type.perform(context),
            {
                "paths_file": paths_file_path,
                "calibration_file": calibration_file_path,
                "canvas_file": canvas_file_path,
            },
            effective_parameters,
            robot_description_xml,
            robot_description_semantic_xml,
            timing,
            preflight_receipt,
            clock_monitor,
            qualified_build_identities,
        )
        print(f"Qualification provenance written atomically: {manifest}")

    painting_executor_node = Node(
        package="robross_painter",
        executable="painting_executor",
        name="painting_executor",
        output="screen",
        parameters=parameters,
    )
    return [painting_executor_node]


def generate_launch_description():
    default_calibration = os.path.join(
        get_package_share_directory("robross_painter"), "config", "rviz_wall_a4.yaml"
    )
    declared_arguments = [
        DeclareLaunchArgument(
            "aubo_type",
            default_value="aubo_i5",
            description="Robot model (must match the running control/moveit stack).",
        ),
        DeclareLaunchArgument(
            "paths_file",
            description="Absolute path to a RobRoss painting_paths.json file.",
        ),
        DeclareLaunchArgument(
            "calibration_file",
            default_value=default_calibration,
            description="YAML with canvas pose / heights / speed parameters.",
        ),
        DeclareLaunchArgument(
            "canvas_file",
            default_value="",
            description="Optional taught canvas pose YAML from "
            "teach_canvas.py; overrides the canvas pose in calibration_file.",
        ),
        DeclareLaunchArgument(
            "cartesian_failure_artifact_dir",
            default_value="",
            description="Existing writable directory for planning-failure artifacts.",
        ),
        DeclareLaunchArgument(
            "cartesian_capture_command_index",
            default_value="0",
            description="Dry-run-only command index to capture as a probe template.",
        ),
        DeclareLaunchArgument(
            "require_qualified_build",
            default_value="false",
            description="Fail launch unless exact source-built driver/controller "
            "identities are present.",
        ),
        DeclareLaunchArgument(
            "qualification_run",
            default_value="false",
            description="Require host-time evidence and provenance before executor start.",
        ),
        DeclareLaunchArgument("session_id", default_value=""),
        DeclareLaunchArgument("provenance_dir", default_value=""),
        DeclareLaunchArgument("preflight_receipt", default_value=""),
        DeclareLaunchArgument("clock_monitor_file", default_value=""),
        DeclareLaunchArgument("controller_update_rate_hz", default_value="125"),
        DeclareLaunchArgument("servoj_time", default_value="0.008"),
        DeclareLaunchArgument("rtde_state_rate", default_value="200"),
        DeclareLaunchArgument("rtde_state_max_age", default_value="0.05"),
        DeclareLaunchArgument(
            "expected_driver_repository",
            default_value="https://github.com/dwarfguard/aubo_ros2_driver.git",
            description="Exact driver source repository required for qualification.",
        ),
        DeclareLaunchArgument(
            "expected_driver_commit",
            default_value="",
            description="Full immutable driver commit containing the installed "
            "qualification identity mechanism.",
        ),
        DeclareLaunchArgument(
            "expected_controller_repository",
            default_value="",
            description="Exact ros2_controllers fork required for qualification.",
        ),
        DeclareLaunchArgument(
            "expected_controller_commit",
            default_value="",
            description="Full immutable ros2_controllers commit required for qualification.",
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
