import os
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
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
    parameters.append({"paths_file": paths_file_path})

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
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
