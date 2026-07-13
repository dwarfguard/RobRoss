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
| `paint_path` | Continuous polyline at contact height: all points become waypoints of one Cartesian trajectory, retimed as a whole, so the pen draws through corners without stopping. Densely sampled curves execute the same way. |
| `lift_tool` | Straight ascent to safe height. |

Safety checks refuse out-of-canvas coordinates, `move_to` with the pen down,
strokes with the pen up, and abort on any planning/execution failure. Cartesian
trajectories are retimed with TOTG using the configured velocity and
acceleration scaling.

The canvas does not have to be horizontal: its frame (x right, y down,
z into the paper) can be posed anywhere in `base_link` — flat on a table,
taped to a wall, or on a slanted stand. All motions (hover, descent,
strokes) work along the canvas normal. The pen tip is decoupled from
`ee_link` through a configurable tool offset so the custom pen claw can be
used; targets are pen-tip poses, converted to `ee_link` poses before
planning.

Before executing any command, the node inserts collision objects into the
MoveIt planning scene and refuses to run if any of them cannot be added:

- a large ground plane (top at `ground_z_m` in `base_link`) so no arm link
  can swing below the mounting surface;
- a canvas backing plane just behind the paper (wall / stand board),
  oriented with the canvas frame, so nothing can push past the drawing
  plane (`canvas_backing_enabled`);
- optionally a stand-in box for the pen claw attached to `ee_link`
  (`claw_collision_size_xyz`), since the claw is not part of the Aubo URDF
  and would otherwise be invisible to collision checking.

Joint-space planning and Cartesian path generation both check against
these, so offending trajectories are rejected instead of executed.

It also publishes markers on `robross_markers` so RViz can show the paper
outline and completed strokes.

## Canvas Calibration

The launch default is `config/rviz_wall_a4.yaml`: an A4 sheet on a virtual
wall with a simulated pen/claw offset. `config/demo_v1_rviz.yaml` preserves
the earlier flat-paper fake-hardware setup, and
`config/hardware_wall_a4.yaml` is the real-arm wall template. These files
hold robot-specific settings that intentionally do not belong in the
artwork configs:

| Parameter | Meaning |
| --- | --- |
| `canvas_origin_xyz` | Paper top-left corner in `base_link`, on the paper surface (meters). |
| `canvas_quat_xyzw` | Full canvas orientation (x right, y down, z into the paper). Written by `teach_canvas.py`; when present it wins over `canvas_x_yaw_deg`. |
| `canvas_x_yaw_deg` | Legacy flat-paper fallback: direction of the page x axis in the base XY plane. |
| `safe_clearance_m` | Hover distance off the paper for travel moves, along the canvas normal. |
| `ground_z_m` | Top of the ground collision plane in `base_link` z; slightly negative by default so the robot base is not flagged as colliding. |
| `canvas_backing_enabled`, `canvas_backing_clearance_m` | Collision plane behind the paper (wall/board) and its gap to the drawing plane. |
| `tool_offset_xyz`, `tool_offset_rpy` | Pen-tip frame in `ee_link` — where the custom claw holds the pen. Zero means `ee_link` is the pen tip. |
| `tool_spin_deg` | Rotation of the claw about the pen axis; choose one that keeps the claw clear of the wrist and wall. |
| `claw_collision_size_xyz`, `claw_collision_offset_xyz` | Stand-in collision box for the claw, attached to `ee_link`. Size `[0,0,0]` disables it. |
| `velocity_scaling`, `acceleration_scaling`, `eef_step_m`, `dry_run` | Motion execution settings. |
| `cartesian_jump_threshold` | Rejects Cartesian segments whose IK flips arm configuration between samples (the flip would execute as an unchecked sweep through the robot/ground/wall). Never 0 — that disables the check. Travel moves that fail it are replanned in joint space; pen-down moves abort. |

With `dry_run: true`, the executor sends no trajectory goals. It still plans
the complete command sequence by carrying each successful trajectory's final
joint state forward as the start state of the next plan. This validates the
actual approach, descent, stroke, lift, and travel transitions rather than
planning every target independently from the robot's unchanged current pose.

## Teaching the Canvas Pose (Real Hardware)

`teach_canvas.py` measures the paper pose directly with the arm instead of
hand-measuring it. It works for a wall, a table, or a slanted stand.

1. Start the driver on the real arm so TF `base_link -> ee_link` is
   published, and put the arm in the pendant's freedrive / hand-guide mode.
2. Run the teach node with the same tool offset the executor will use:

   ```bash
   ros2 run robross_painter teach_canvas.py --ros-args \
     -p tool_offset_xyz:="[0.0, 0.0, 0.12]" \
     -p output_file:=$HOME/canvas_calibration.yaml
   ```

3. Touch the pen tip to the paper's corners (top-left / top-right /
   bottom-left of the artwork as it should appear) and record each:

   ```bash
   ros2 service call /teach_canvas/record_top_left std_srvs/srv/Trigger
   ros2 service call /teach_canvas/record_top_right std_srvs/srv/Trigger
   ros2 service call /teach_canvas/record_bottom_left std_srvs/srv/Trigger
   ros2 service call /teach_canvas/save std_srvs/srv/Trigger
   ```

   `save` reports the measured paper size, warns if it deviates from A4 by
   more than 5 mm or the corners are off square, and writes a parameter
   file with `canvas_origin_xyz` + `canvas_quat_xyzw`.
4. Launch the executor with the wall profile plus the taught pose:

   ```bash
   ros2 launch robross_painter paint.launch.py \
     calibration_file:=<path>/hardware_wall_a4.yaml \
     canvas_file:=$HOME/canvas_calibration.yaml \
     paths_file:=$ROBROSS_REPO/output/test_line_paths.json
   ```

   Keep `dry_run: true` (the wall template's default) for the first pass,
   then flip it and start with the 50 mm test line at low speed.

## Run The Default Wall Setup In RViz

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

In RViz, add a `Marker` display on topic `robross_markers` to see the vertical
paper outline and strokes. The default `paint.launch.py` calibration is
`rviz_wall_a4.yaml`, so no calibration argument is needed for this setup.

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

Optional arguments: `calibration_file:=<yaml>` overrides the canvas pose,
heights, and speeds; `canvas_file:=<yaml>` layers a taught canvas pose from
`teach_canvas.py` on top of it.

## Flat-Paper Test In RViz

To use the earlier horizontal-paper setup instead of the default wall, pass
`config/demo_v1_rviz.yaml` explicitly. With Terminals 1 and 2 from the
previous section running:

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=aubo_i5 \
  paths_file:=$ROBROSS_REPO/output/test_line_paths.json \
  calibration_file:=$(ros2 pkg prefix robross_painter)/share/robross_painter/config/demo_v1_rviz.yaml
```

The Marker display shows the paper outline lying horizontally. Repeat with
`output/painting_paths.json` for the full artwork.

Gotchas:

- Start from a non-colliding pose. If a previous flat-paper run left the arm
  reaching past x = 0.55, planning refuses (start state inside the wall) —
  restart Terminal 1 to reset the fake arm to home.
- If the executor hangs at startup while adding collision objects, restart
  the whole stack (stale DDS state from killed nodes).

## Implementation Note

Do not spin the executor node in an external executor. In MoveIt Humble,
`MoveGroupInterface` spins the passed node internally, and a second executor
can steal its action responses. For the same reason the code avoids
`getCurrentState()` and builds the retiming start state from the trajectory's
first waypoint.
