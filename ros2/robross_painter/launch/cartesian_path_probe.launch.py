import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def require_file(file_path, label):
    resolved_path = os.path.abspath(os.path.expanduser(file_path))
    if not os.path.isfile(resolved_path):
        raise RuntimeError(f"{label} is not a file: {resolved_path}")
    return resolved_path


def positive_integer(value, label):
    try:
        parsed = int(value)
    except ValueError as error:
        raise RuntimeError(f"{label} must be a positive integer") from error
    if parsed <= 0:
        raise RuntimeError(f"{label} must be a positive integer")
    return parsed


def required_confirmation(value):
    if value.lower() not in {"true", "1", "yes"}:
        raise RuntimeError(
            "confirm_isolated_move_group must be true; the probe temporarily "
            "reconstructs and then restores its owned planning-scene objects"
        )
    return True


def launch_setup(context, *args, **kwargs):
    request_file = require_file(
        LaunchConfiguration("recorded_request_file").perform(context),
        "recorded_request_file",
    )
    start_state_value = LaunchConfiguration("start_state_file").perform(context)
    start_state_file = (
        require_file(start_state_value, "start_state_file")
        if start_state_value
        else ""
    )
    repetitions = positive_integer(
        LaunchConfiguration("repetitions").perform(context), "repetitions"
    )
    isolated = required_confirmation(
        LaunchConfiguration("confirm_isolated_move_group").perform(context)
    )
    return [
        Node(
            package="robross_painter",
            executable="cartesian_path_probe",
            name="cartesian_path_probe",
            output="screen",
            parameters=[
                {
                    "recorded_request_file": request_file,
                    "start_state_file": start_state_file,
                    "repetitions": repetitions,
                    "allow_unexpected_scene_objects": False,
                    "confirm_isolated_move_group": isolated,
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "recorded_request_file",
                description="Cartesian failure artifact produced by painting_executor.",
            ),
            DeclareLaunchArgument(
                "start_state_file",
                default_value="",
                description="Optional bag-derived start-state seed JSON.",
            ),
            DeclareLaunchArgument(
                "repetitions",
                default_value="20",
                description="Number of planning-only repetitions.",
            ),
            DeclareLaunchArgument(
                "confirm_isolated_move_group",
                default_value="false",
                description="Required acknowledgement that replay uses an isolated move_group.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
