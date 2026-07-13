import os

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
    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("aubo_moveit_config"), "config", "aubo_robot.srdf"]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content.perform(
            context
        )
    }
    kinematics_yaml = load_yaml("aubo_moveit_config", "config/kinematics.yaml")

    parameters = [
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        calibration_file.perform(context),
    ]
    # Taught canvas pose (teach_canvas.py output) layered after the base
    # calibration so its canvas_origin_xyz / canvas_quat_xyzw win.
    canvas_file_path = canvas_file.perform(context)
    if canvas_file_path:
        parameters.append(canvas_file_path)
    parameters.append({"paths_file": paths_file})

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
