# robross_painter

ROS 2 and MoveIt adapter for RobRoss path files. It supports Aubo i5
fake-hardware testing in RViz and calibrated real-arm operation.

## Documentation

| Document | Purpose |
| --- | --- |
| This README | Build context, RViz workflow, and canvas teaching |
| [Configuration reference](REFERENCE.md) | Profiles, parameters, collision model, and motion checks |
| [Hardware preflight](PREFLIGHT.md) | Required real-arm procedure and abort rules |
| [Path format](../../docs/painting-paths-format.md) | Canvas coordinates and command schema |

The repository [root README](../../README.md) contains the workspace import and
build instructions. This package is built from
`RobRoss/ros2/robross_painter` as a normal colcon package alongside the Aubo
driver and MoveIt configuration.

## Path Execution

The executor converts canvas-space commands into pen-tip poses:

| Command | Behavior |
| --- | --- |
| `select_tool`, `dip_paint` | Logged no-op for the pen prototype. |
| `move_to` | Pen-up travel; the first move uses nearby joint-space IK. |
| `lower_tool` | Straight motion along the canvas normal to contact. |
| `paint_stroke` | Straight contact motion. |
| `paint_path` | One continuous contact trajectory through every polyline point. |
| `lift_tool` | Straight retreat along the canvas normal. |

Canvas coordinates are in millimeters from the paper's top-left, with `x`
right and `y` down. The canvas may be horizontal, vertical, or slanted. Its
calibrated frame maps those coordinates into `base_link`; tool-offset settings
then convert pen-tip targets into `ee_link` targets.

## Safety Model

Before execution, the node adds the configured ground, canvas backing, and claw
collision geometry. It then applies these fail-closed checks:

- fresh measured joint state before motion and after execution;
- current-state-seeded IK for a nearby first approach;
- configured elbow-family limits at startup and along every trajectory;
- bounded goal displacement, total travel, and sample steps for guarded joints;
- tighter guarded-joint travel while lowering, painting, or lifting;
- Cartesian jump detection and post-retiming pen-tip path validation;
- MoveIt bounds and collision checks at interpolated trajectory samples;
- measured pen-tip endpoint checks after execution.

The executor does not move through an automatic home pose, switch elbow family,
or retry an unconstrained IK goal. If contact-state execution becomes uncertain,
it attempts only a measured straight retreat from the canvas.

See [REFERENCE.md](REFERENCE.md) for the exact controls. Do not weaken a safety
limit merely to make a rejected trajectory execute.

## Calibration Profiles

| Profile | Use |
| --- | --- |
| `config/rviz_wall_a4.yaml` | Default fake-hardware A4 wall. Simulation only. |
| `config/rviz_taught_a4.yaml` | Fake-hardware tests with a taught canvas on any plane (slanted, ground). No ground collision plane, auto-sized backing patch, relaxed base-axis guards. Simulation only. |
| `config/demo_v1_rviz.yaml` | Earlier fake-hardware horizontal-paper setup. |
| `config/hardware_a4.yaml` | Real-arm profile for any taught surface. Source of truth for the measured tool offset and claw collision box; the sim profiles must carry the same values. |

`paint.launch.py` defaults to `rviz_wall_a4.yaml`. Never use that default on a
real arm. A real-arm launch must explicitly pass both `calibration_file` and a
taught `canvas_file`. The launch fails before starting the executor if any
supplied file is missing or if the calibration/canvas YAML has the wrong role.

## Run In RViz

Generate the path files first by following the
[Mondrian pipeline guide](../../Image_Process/mondrian/README.md). From the
colcon workspace root:

```bash
colcon build --packages-select robross_painter
```

Each terminal below is opened at the workspace root. Source the workspace in
every terminal, and export `ROBROSS_REPO` in each terminal that references it
(Terminal 3 and the real-arm dry run):

```bash
source install/setup.bash
export ROBROSS_REPO=$PWD/src/RobRoss
```

Terminal 1, start fake controllers. The `aubo_ros2_driver` and
`aubo_moveit_config` packages come from the Aubo fork imported through
`ros2/robross_aubo.repos` during the [workspace setup](../../README.md):

```bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_i5 \
  use_fake_hardware:=true
```

Terminal 2, start MoveIt and RViz:

```bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=aubo_i5
```

Terminal 3, run the full artwork with the default virtual wall
(`config/rviz_wall_a4.yaml`):

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=aubo_i5 \
  paths_file:=$ROBROSS_REPO/output/painting_paths.json
```

Use `output/test_line_paths.json` instead for the 50 mm line, or
`output/curve_test_paths.json` for the post-contact curves and corners test.
Add an RViz `Marker` display on `robross_markers` to see the paper outline and
completed strokes.

For the horizontal-paper simulation, pass the legacy profile as an extra
argument to the Terminal 3 launch:

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=aubo_i5 \
  paths_file:=$ROBROSS_REPO/output/painting_paths.json \
  calibration_file:=$(ros2 pkg prefix robross_painter)/share/robross_painter/config/demo_v1_rviz.yaml
```

## Teach A Real Canvas

Complete the [hardware preflight](PREFLIGHT.md) in order; this section only
documents the teaching tool.

1. Calibrate the robot model as described by the maintained Aubo driver.
2. Create a working copy of `hardware_a4.yaml`, re-verify the measured values, and keep
   `dry_run: true` for the initial full-artwork plan:

```bash
cp "$(ros2 pkg prefix robross_painter)/share/robross_painter/config/hardware_a4.yaml" \
  "$HOME/hardware_a4.yaml"
grep -n "dry_run: true" "$HOME/hardware_a4.yaml"
```

The calibration command creates `aubo_i5_calibrated.urdf`. Rebuild
`aubo_description`, then use the corresponding model name in every launch:

```bash
cd ~/robross_aubo_ws
python3 src/aubo_ros2_driver/aubo_description/scripts/calibrate_urdf_dh.py \
  --robot-model aubo_i5 \
  --robot-ip <robot-ip>
colcon build --packages-select aubo_description
source install/setup.bash
export AUBO_TYPE=aubo_i5_calibrated
```

Using `aubo_i5` after calibration silently selects the stock, uncalibrated
model. The control, MoveIt, and painter launches must all receive the same
`aubo_type:=$AUBO_TYPE` value.

Start the real stack in separate terminals before teaching or painting:

```bash
# Terminal 1
source ~/robross_aubo_ws/install/setup.bash
export AUBO_TYPE=aubo_i5_calibrated
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=$AUBO_TYPE robot_ip:=<robot-ip> use_fake_hardware:=false

# Terminal 2
source ~/robross_aubo_ws/install/setup.bash
export AUBO_TYPE=aubo_i5_calibrated
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=$AUBO_TYPE
```

3. Start the real driver with `aubo_type:=$AUBO_TYPE`, then release its
   position controller before enabling pendant freedrive. This keeps the
   joint-state broadcaster and TF active while stopping servo-position
   commands:

```bash
ros2 control switch_controllers \
  --deactivate joint_trajectory_controller --strict
ros2 control list_controllers
```

Confirm that `joint_trajectory_controller` is `inactive` and
`joint_state_broadcaster` remains `active`. Then run the teaching node with
the exact measured tool offset from that hardware profile and the intended
pen preload as `plane_bias_mm`, plus the nudge helper in another terminal
(it needs the Terminal 2 `move_group` and reports "ready" once connected):

```bash
ros2 run robross_painter teach_canvas.py --ros-args \
  -p tool_offset_xyz:="[<x>, <y>, <z>]" \
  -p plane_bias_mm:=1.8 \
  -p output_file:=$HOME/canvas_calibration.yaml

# Second terminal: sub-millimeter approach steps along the pen axis. Launch
# (not `ros2 run`) so its MoveGroupInterface gets the robot_description/SRDF;
# aubo_type must match the running stack.
ros2 launch robross_painter teach_nudge.launch.py \
  aubo_type:=aubo_i5 \
  tool_offset_rpy:="[<r>, <p>, <y>]"
```

Teach at **just-touch**: the recorded point is the free-length virtual pen
tip, so any spring compression at record time pushes the taught plane that
far behind the paper — the drawing preload is applied in software by
`plane_bias_mm` instead (see PREFLIGHT.md section 2). Touch the **physical
paper corners**, not the artwork-margin corners. For each corner:

1. Enable pendant freedrive and bring the pen tip to hover a few mm off
   the corner, roughly perpendicular to the paper — the i5's freedrive
   breakaway force is too high for accurate millimeter motions, so stop
   there.
2. Disable freedrive and reactivate the controller
   (`ros2 control switch_controllers --activate joint_trajectory_controller
   --strict`). Before the first corner only, verify the nudge direction
   well clear of the paper: `~/nudge_out` must move the pen away from it.
3. Step the pen in until the pen body **first visibly moves** relative to
   the claw — that is the paper surface at zero compression; stop there:

   ```bash
   ros2 service call /teach_nudge/nudge_in std_srvs/srv/Trigger
   ros2 param set /teach_nudge nudge_step_mm 0.2   # finer steps for the last mm
   ```

4. Record the corner (hands are already off the arm; each record averages
   the pen tip over the last second and is rejected if the arm was still
   moving — wait and call it again).
5. `~/nudge_out` a few steps to clear the paper, deactivate the controller
   again, and freedrive to the next corner.

Record top-left, top-right, bottom-left, then bottom-right as a validation
point:

```bash
ros2 service call /teach_canvas/record_top_left std_srvs/srv/Trigger
ros2 service call /teach_canvas/record_top_right std_srvs/srv/Trigger
ros2 service call /teach_canvas/record_bottom_left std_srvs/srv/Trigger
ros2 service call /teach_canvas/record_bottom_right std_srvs/srv/Trigger
ros2 service call /teach_canvas/save std_srvs/srv/Trigger
```

`save` writes `canvas_origin_xyz` (the top-left corner pushed `plane_bias_mm`
behind the paper along the canvas normal) and `canvas_quat_xyzw`. The
optional bottom-right corner never changes the saved pose; `save` warns when
it lies more than 2 mm from where the other three corners predict it. Re-teach
if the reported dimensions differ materially from A4, the corners are not
square, or the bottom-right residual warning appears.

Disable freedrive on the pendant before returning control to ROS, then
reactivate the trajectory controller. The driver resumes from the measured
joint pose rather than the pre-teach command pose:

```bash
ros2 control switch_controllers \
  --activate joint_trajectory_controller --strict
```

The driver rejects controller activation while freedrive is still enabled.
Never attempt to use freedrive while `joint_trajectory_controller` is active.

First dry-run the **complete artwork** with the reviewed hardware profile and
taught canvas:

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=$AUBO_TYPE \
  calibration_file:=$HOME/hardware_a4.yaml \
  canvas_file:=$HOME/canvas_calibration.yaml \
  paths_file:=$ROBROSS_REPO/output/painting_paths.json
```

Only after that succeeds should a reviewed profile set `dry_run: false` and run
`output/test_line_paths.json` at the preflight speeds. Keep an operator on the
e-stop.

## Troubleshooting

- Start from a collision-free, approved elbow-up posture. The executor will not
  reposition an invalid start automatically.
- `IK goal moves <joint> by N deg (limit M deg)`: the taught canvas sits
  outside the guarded base-swing range of the active profile. In simulation use
  `rviz_taught_a4.yaml`; on hardware, place the paper within the guarded range
  instead of raising the limits.
- `No bounded elbow-up joint-space plan found` with a ground-lying or low
  canvas: the ground collision plane is blocking the approach. Use a profile
  with `ground_enabled: false` so the auto-sized backing patch protects the
  surface under the paper instead. The executor removes its prior ground object
  when applying the disabled profile; a `Ground collision plane disabled and
  absent` log confirms the update.
- The executor logs only to its own file; after a silent exit check
  `~/.ros/log/painting_executor_<pid>_<stamp>.log`.
- Restart the whole simulation stack after stale DDS state or a planning-scene
  reset. Restart the painting rather than resuming midway.
- In MoveIt Humble, do not spin the executor node in a second external executor;
  `MoveGroupInterface` manages the passed node's action responses.
- The independent `/joint_states` monitor measures freshness at receipt time so
  fake controllers with zero message timestamps remain supported.
