# robross_painter

Aubo i5 adapter for the RobRoss project. It executes a `painting_paths.json`
file through MoveIt on the `aubo_ros2_driver` stack. It works against fake
hardware for the RViz milestone and can later run against the real arm after
physical calibration.

## Workspace Context

This package is source-controlled in `RobRoss/ros2/robross_painter`, but it is
built as a normal ROS 2 package inside a colcon workspace:

```text
robross_aubo_ws/
  src/
    RobRoss/
      ros2/robross_painter/
    aubo_ros2_driver/
```

The RobRoss root README contains the full workspace setup using
`ros2/robross_aubo.repos`.

## What It Does

`painting_executor` reads the RobRoss path file (mm, origin top-left, y down)
and maps each command to robot motion:

| Command | Motion |
| --- | --- |
| `select_tool`, `dip_paint` | No-op for pen Demo v1, logged. |
| `move_to` | Travel at safe height; first one joint-space, rest straight Cartesian. |
| `lower_tool` | Straight descent to pen-contact height. |
| `paint_stroke` | Straight Cartesian line at contact height. |
| `lift_tool` | Straight ascent to safe height. |

Safety checks refuse out-of-canvas coordinates, `move_to` with the pen down,
strokes with the pen up, and abort on any planning/execution failure. Cartesian
trajectories are retimed with TOTG using the configured velocity and
acceleration scaling.

It also publishes markers on `robross_markers` so RViz can show the paper
outline and completed strokes.

## Canvas Calibration

`config/demo_v1_rviz.yaml` holds robot-specific settings that intentionally do
not belong in the artwork configs:

| Parameter | Meaning |
| --- | --- |
| `canvas_origin_xyz` | Paper top-left corner in `base_link`; z is pen-contact height. |
| `canvas_x_yaw_deg` | Direction of the page x axis in the base XY plane. |
| `safe_clearance_m` | Hover height for travel moves. |
| `tool_rpy` | Pen-down orientation of `ee_link`. |
| `velocity_scaling`, `acceleration_scaling`, `eef_step_m`, `dry_run` | Motion execution settings. |

For the real robot, replace origin/yaw with measured values from taught paper
corners and lower speed scaling before contact tests.

## Run In RViz With Fake Hardware

From the colcon workspace root:

```bash
export ROBROSS_REPO=$PWD/src/RobRoss
source install/setup.bash
```

Terminal 1, start controllers with fake hardware:

```bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_i5 \
  use_fake_hardware:=true
```

Terminal 2, start MoveIt and RViz:

```bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=aubo_i5
```

In RViz, add a `Marker` display on topic `robross_markers` to see the paper
outline and strokes.

Terminal 3, execute the generated RobRoss path file:

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=aubo_i5 \
  paths_file:=$ROBROSS_REPO/output/painting_paths.json
```

The 50 mm first-contact test line uses the same executor:

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=aubo_i5 \
  paths_file:=$ROBROSS_REPO/output/test_line_paths.json
```

Optional argument: `calibration_file:=<yaml>` overrides the canvas pose,
heights, and speeds.

## Implementation Note

Do not spin the executor node in an external executor. In MoveIt Humble,
`MoveGroupInterface` spins the passed node internally, and a second executor
can steal its action responses. For the same reason the code avoids
`getCurrentState()` and builds the retiming start state from the trajectory's
first waypoint.
