"""Launch the teach-time pen-axis nudge helper with a robot model.

teach_nudge builds a MoveGroupInterface, which needs robot_description (URDF)
and robot_description_semantic (SRDF) on its node to construct the robot model
— exactly like painting_executor gets them from paint.launch.py. Running the
node bare with `ros2 run` supplies neither, so MoveGroupInterface fails with
"Unable to parse SRDF / Unable to construct robot model". This launch file
assembles both the same way paint.launch.py does and hands them to the node.

Needs move_group already running (Terminal 2 in the RViz flow / the real-arm
MoveIt launch). Kinematics is intentionally omitted: computeCartesianPath runs
IK server-side in move_group; the node only needs URDF+SRDF locally.
"""

import yaml
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


def launch_setup(context, *args, **kwargs):
    aubo_type = LaunchConfiguration("aubo_type")

    # Same robot_description the driver/moveit/executor launches build, so the
    # MoveGroupInterface in teach_nudge sees an identical model.
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
    robot_description = {
        "robot_description": robot_description_content.perform(context)
    }
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
        "robot_description_semantic": robot_description_semantic_content.perform(context)
    }

    # tool_offset_rpy is a double array; parse the "[r, p, y]" string arg.
    rpy_raw = yaml.safe_load(LaunchConfiguration("tool_offset_rpy").perform(context))
    if not isinstance(rpy_raw, (list, tuple)) or len(rpy_raw) != 3:
        raise RuntimeError(
            f"tool_offset_rpy must be a 3-element list, got {rpy_raw!r}"
        )
    node_params = {
        "tool_offset_rpy": [float(value) for value in rpy_raw],
        "nudge_step_mm": float(
            LaunchConfiguration("nudge_step_mm").perform(context)
        ),
        "velocity_scaling": float(
            LaunchConfiguration("velocity_scaling").perform(context)
        ),
        "acceleration_scaling": float(
            LaunchConfiguration("acceleration_scaling").perform(context)
        ),
        "jump_threshold": float(
            LaunchConfiguration("jump_threshold").perform(context)
        ),
        "ee_frame": LaunchConfiguration("ee_frame").perform(context),
    }

    teach_nudge_node = Node(
        package="robross_painter",
        executable="teach_nudge",
        name="teach_nudge",
        output="screen",
        parameters=[robot_description, robot_description_semantic, node_params],
    )
    return [teach_nudge_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "aubo_type",
            default_value="aubo_i5",
            description="Robot model (must match the running control/moveit stack).",
        ),
        DeclareLaunchArgument(
            "tool_offset_rpy",
            default_value="[0.0, 0.0, 0.0]",
            description="Pen axis as roll/pitch/yaw applied to ee +Z; must match "
            "the executor's tool_offset_rpy.",
        ),
        DeclareLaunchArgument(
            "nudge_step_mm",
            default_value="0.5",
            description="Step size per nudge, mm (clamped to [0.05, 2.0]).",
        ),
        DeclareLaunchArgument(
            "velocity_scaling",
            default_value="0.05",
            description="MoveIt velocity scaling for the nudge motion.",
        ),
        DeclareLaunchArgument(
            "acceleration_scaling",
            default_value="0.05",
            description="MoveIt acceleration scaling for the nudge motion.",
        ),
        DeclareLaunchArgument(
            "jump_threshold",
            default_value="2.0",
            description="Cartesian jump threshold (never 0).",
        ),
        DeclareLaunchArgument(
            "ee_frame",
            default_value="ee_link",
            description="Frame whose +Z (rotated by tool_offset_rpy) is the pen axis.",
        ),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
