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
- Cartesian jump detection and post-retiming pen-tip path validation, with
  the canvas-normal deviation checked separately (and much tighter) than the
  tangential deviation;
- Cartesian trajectories sent position-only at the controller period
  (`controller_sample_dt`), so the controller's linear interpolation matches
  exactly what the validator checked (remediation plan Phase 1);
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

## Teach The Pen-Tip TCP (Pin Calibration)

The `tool_offset_xyz` / `tool_offset_rpy` in the calibration profiles set where
the pen tip is relative to `ee_link`. A hand-measured value is only good to a
millimetre or two; `teach_tcp.py` measures it properly with a **sharp
calibration pin** using the classic pivot ("N-point") method, and every taught
canvas and stroke depends on it — so do this **before** teaching the canvas, and
redo it after any pen or claw change.

Bring up the real stack and release the position controller exactly as in *Teach
A Real Canvas* below (the node only needs live `base_link -> ee_link` TF; it
needs no tool offset and no `move_group`). Then, with a sharp pin clamped
rigidly and pointing up inside the arm's reach:

```bash
ros2 run robross_painter teach_tcp.py --ros-args \
  -p output_file:=$HOME/tcp_calibration.yaml

# Second terminal: the same sub-millimeter nudge helper used for the canvas.
ros2 launch robross_painter teach_nudge.launch.py aubo_type:=$AUBO_TYPE
```

**Part A — tip position.** Touch the pen tip to the pin tip from **four or more
widely varied wrist orientations** (freedrive to hover, then `~/nudge_in` to
just-touch — same doctrine as the canvas). Record each, and reorient the wrist a
lot between touches (near-identical orientations make the solve ill-conditioned):

```bash
ros2 service call /teach_tcp/record_tip std_srvs/srv/Trigger
ros2 service call /teach_tcp/solve std_srvs/srv/Trigger   # check the tip scatter
```

`~/solve` prints the tip-scatter RMS; keep adding varied touches until it is
below `residual_warn_mm` (default 0.7 mm).

**Part B — pen axis** (`tool_offset_rpy`), optional but recommended if the pen
sits angled in the claw. Primary method: with the tip on the pin, hold the pen
**plumb** (a claw/barrel flat against a small bubble level) and call
`~/record_axis_vertical` a few times. Higher-accuracy alternative if you have a
second identifiable point on the pen centerline: run a second pivot on it with
`~/record_axis_point` (beware touching the side of a bare barrel — that offsets
by its radius). With neither, the axis stays `[0, 0, 0]` (pen parallel to ee +Z).

```bash
ros2 service call /teach_tcp/record_axis_vertical std_srvs/srv/Trigger
ros2 service call /teach_tcp/save std_srvs/srv/Trigger
```

`~/save` writes a `painting_executor` parameter fragment with the measured
`tool_offset_xyz` / `tool_offset_rpy` and a report (touch count, scatter, pin
point, axis tilt). Then:

1. Copy both values into **all four** profiles (`hardware_a4.yaml`,
   `rviz_wall_a4.yaml`, `rviz_taught_a4.yaml`, `demo_v1_rviz.yaml`) — they must
   stay identical.
2. **Re-pick `tool_spin_deg`** by eye for claw/cable clearance (it is a separate
   clearance choice, not calibrated here).
3. **Re-teach the canvas** — any existing `canvas_calibration.yaml` was recorded
   with the old offset and is now stale.

Other services: `~/record_axis_point`, `~/clear` (reset all touches).

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

Record all four corners, then ~5-9 interior sample points (same just-touch
procedure) spread across the paper — a rough 3×3 (center, mid-edges, quarter
points):

```bash
ros2 service call /teach_canvas/record_top_left std_srvs/srv/Trigger
ros2 service call /teach_canvas/record_top_right std_srvs/srv/Trigger
ros2 service call /teach_canvas/record_bottom_left std_srvs/srv/Trigger
ros2 service call /teach_canvas/record_bottom_right std_srvs/srv/Trigger
ros2 service call /teach_canvas/record_sample std_srvs/srv/Trigger   # repeat x5-9
ros2 service call /teach_canvas/save std_srvs/srv/Trigger
```

`save` writes `canvas_origin_xyz` (the top-left corner pushed `plane_bias_mm`
behind the paper along the canvas normal), `canvas_quat_xyzw`, and
`canvas_z_correction_coeffs`. All four corners feed the least-squares plane fit
(so no single noisy corner tips the plane); the interior samples fit a smooth
quadratic Z-correction surface that is **recorded as a flatness diagnostic
only — the executor does NOT apply it during motion**
(`docs/aubo-painting-tracking-remediation-plan.md` Section 4 forbids
position-dependent Z compensation; the executor always uses the flat taught
plane). The fit measures the reach-dependent, non-planar contact error a
single plane cannot represent. `save` reports the out-of-plane error before
and after the fitted surface,
warns above `flatness_warn_mm` (0.3 mm), and **refuses** above
`flatness_refuse_mm` (0.6 mm — add interior samples or re-teach). It still warns
when bottom-right lies more than 2 mm from where the other three predict it.
Re-teach if the reported dimensions differ materially from A4 or any warning
appears.

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

## Offline Tracking-Bag Analysis

`scripts/analyze_tracking_bag.py` is the Phase 0 tool from
`docs/aubo-painting-tracking-remediation-plan.md`: it turns a recorded painting
run into per-command tracking metrics so every timing/interpolation change can
be compared against the same baseline. It is strictly read-only — it never
initializes ROS, creates no node, and publishes nothing; it only reads the bag
files.

```bash
ros2 run robross_painter analyze_tracking_bag.py <bag_dir> \
  --canvas-file $HOME/canvas_calibration.yaml \
  --calibration-file $HOME/hardware_a4.yaml \
  --plane-bias-mm 1.0 \
  --csv tracking.csv \
  --servoj-csv servoj.csv   # optional; only meaningful for Phase 2 driver bags
```

Required bag topics: `/joint_trajectory_controller/controller_state` (reference
and feedback joints), `/rosout` (painting_executor command labels drive the
segmentation), and `/robot_description` (the runtime URDF used for forward
kinematics — the same calibrated chain MoveIt used; pass `--urdf` if the bag
lacks it). For each command segment it reports stroke direction, speed, signed
canvas-normal error (positive = into the paper), tangential error, estimated
spring compression (`plane_bias_mm + actual canvas z`), per-joint errors, and
publication rate/jitter for controller state and joint states. `--csv` exports
per-sample rows for offline plotting.

### ServoJ timing (Phase 2)

When the bag also carries the Aubo driver's ServoJ diagnostics — the
`aubo_servoj_diag` `/rosout` lines emitted by the Phase 2A instrumentation
(`servoj_config` once at activation, one `servoj_stats` line per report window)
— the summary automatically gains a **ServoJ timing** section: the effective
control-loop rate (and its percentage of the configured rate), `servoJoint`
RPC and whole-`Servoj` durations, late-cycle runs, queue-full events/retries,
and the servoJoint return-code breakdown, aggregated across the whole bag. It
ends with a **Phase 2B timing gate** line summarizing the plan's Section 7
checks (loop rate ≥ 95% of configured, no queue-full, no non-OK return
codes/exceptions, no latched timing fault). The joint-delay portion of that
gate (median < 30 ms / p95 < 50 ms) is assessed separately from the tracking
cross-correlation. `--servoj-csv` writes the per-window timing series, which is
the easiest way to compare two candidate timing trials (e.g. 125 Hz / t=0.008
vs 200 Hz / t=0.005). Bags recorded before Phase 2A, or on fake hardware, carry
no such lines and the section is simply omitted.

Acceptance gate (run on the robot host where the July 22 baseline bags live):

```bash
ros2 run robross_painter analyze_tracking_bag.py \
  ~/robross_aubo_ws/rosbag2_2026_07_22-20_32_44 \
  --canvas-file <the taught canvas yaml used that day> \
  --calibration-file <the hardware_a4.yaml used that day> --plane-bias-mm 1.0
```

The reported mean normal errors must match the remediation plan's Section 2
tables within 0.05 mm and the joint maxima within 0.05 deg.

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
