# Hardware First-Run Guide: Aubo i5 over Ethernet

Step-by-step commands for the first real-arm painting session, from cabling the robot to the
full A4 artwork. Companion to `docs/hardware-test-checklist.md` (what to verify) and
`ros2/robross_painter/PREFLIGHT.md` (run top-to-bottom before every session) — this guide is the
"exact commands" walkthrough; those two remain the authoritative checklists.

## Readiness status (as of 2026-07-23)

**Ready:**
- Workspace built: `install/` contains all packages (`aubo_ros2_driver`, `aubo_moveit_config`,
  `robross_painter`, …).
- Path files generated in `output/`: `painting_paths.json` (38 commands, validated),
  `test_line_paths.json` (the 50 mm first-contact line), and `curve_test_paths.json`
  (the post-contact curves and corners card) + previews.
- `~/hardware_a4.yaml` exists (copy of `ros2/robross_painter/config/hardware_a4.yaml`);
  `tool_offset_xyz: [0.0, -0.0595, 0.0514]` matches the value used during teaching.
- Claw collision box measured on the real claw (2026-07-16 session):
  `claw_collision_size_xyz: [0.02, 0.06, 0.02]`, offset `[0.0, -0.03, 0.0]`. The hardware
  profile is the source of truth for the tool offset and claw box; the sim profiles
  (`rviz_wall_a4.yaml`, `rviz_taught_a4.yaml`) carry the same values.
- `velocity_scaling`/`acceleration_scaling` at 0.1 (correct for first runs);
  `cartesian_jump_threshold: 2.0` (nonzero, correct); `canvas_backing_enabled: true`.
- `dry_run: true` in the shipped profile (flip it only in `~/hardware_a4.yaml`, Step 6).

**Gaps to fix before first contact (in order):**
1. `~/canvas_calibration.yaml` is invalid (measured 366.7 × 315.8 mm, corner skew 23.75°; an A4
   is 210 × 297 mm, skew must be < 2°) → re-teach on the real paper (Step 4). This bad teach —
   not the elbow constraints — is what caused the RViz `wrist3_joint` motion-guard abort.
2. Robot IP unknown — read it off the teach pendant (Settings → Network) once cabled.

## Step 0 — Network (Ethernet direct)

1. Cable the PC to the Aubo control box LAN port. Read the controller IP from the pendant.
2. Put the PC on the same subnet (e.g. controller `192.168.127.128` → PC `192.168.127.100/24`,
   via the desktop network settings or `nmcli`).
3. Verify: `ping <ROBOT_IP>`.

## Step 1 — Every terminal: environment

```bash
export PATH="/usr/bin:$PATH"     # conda shadows ROS python — required or Python nodes crash
cd ~/robross_aubo_ws
source install/setup.bash
export ROBROSS_REPO=$PWD/src/RobRoss
```

## Step 2 — (Recommended, once) URDF calibration from the controller

```bash
python3 -m pip install --user pyaubo-sdk==0.24.1
python3 -c "import numpy, pyaubo_sdk"
python3 src/aubo_ros2_driver/aubo_description/scripts/calibrate_urdf_dh.py \
  --robot-model aubo_i5 --robot-ip <ROBOT_IP>
colcon build --packages-select aubo_description
source install/setup.bash
export AUBO_TYPE=aubo_i5_calibrated
```

Keep `AUBO_TYPE=aubo_i5_calibrated` in every terminal after calibration. Using
`aubo_i5` would silently return to the stock model.

## Step 3 — Bring up the real-arm stack

Terminal 1 (driver, real hardware):
```bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=$AUBO_TYPE robot_ip:=<ROBOT_IP> use_fake_hardware:=false \
  controllers_file:=aubo_controllers.yaml servoj_time:=0.005
```

The standard profile is an explicit matched pair: 200 Hz and ServoJ `t=0.005 s`. Do not change
only one of these values. Step 5.5 defines the separate 125 Hz / 8 ms comparison trial.

Terminal 2 (MoveIt):
```bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=$AUBO_TYPE
```

Sanity check (terminal 3): `ros2 topic echo /joint_states --once` shows live joint angles that
change when the arm is jogged. (`aubo_client.launch.py` is a separate service demo — not needed
for this flow.)

## Step 3.5 — (Recommended, once per pen/claw) Calibrate the pen-tip TCP with the pin

The `tool_offset_xyz` / `tool_offset_rpy` in `hardware_a4.yaml` are hand-measured (good to a
mm or two). For an accurate pen tip, measure them with a sharp calibration pin using the pivot
method — `teach_tcp.py` — **before teaching the canvas** (the canvas is taught in tip coordinates,
so it depends on this offset). It needs only live `base_link -> ee_link` TF (Terminal 1); no
`move_group`, no tool offset. Release the position controller as in Step 4, clamp a sharp pin
pointing up in reach, then:

```bash
ros2 run robross_painter teach_tcp.py --ros-args -p output_file:=$HOME/tcp_calibration.yaml
ros2 launch robross_painter teach_nudge.launch.py aubo_type:=$AUBO_TYPE   # second terminal
```

Touch the pin tip from ≥4 **widely varied** wrist orientations (freedrive to hover, `~/nudge_in`
to just-touch), recording each; check the tip scatter and finish:

```bash
ros2 service call /teach_tcp/record_tip           std_srvs/srv/Trigger   # ×4+, reorient a lot
ros2 service call /teach_tcp/solve                std_srvs/srv/Trigger   # tip scatter < ~0.7 mm
ros2 service call /teach_tcp/record_axis_vertical std_srvs/srv/Trigger   # pen plumb, for the axis
ros2 service call /teach_tcp/save                 std_srvs/srv/Trigger
```

**Gate:** `solve`/`save` report a tip scatter under ~0.7 mm and no near-degenerate warning (if it
fires, reorient the wrist far more between touches). Then copy `tool_offset_xyz`/`tool_offset_rpy`
into **all four** config profiles (keep them identical), re-pick `tool_spin_deg` by eye for
clearance, and proceed to teach the canvas. Full procedure: `ros2/robross_painter/README.md`
("Teach The Pen-Tip TCP"). See details of the tool-offset flow in `hardware_a4.yaml` (Step 3
above). Skip this step only to reuse a previously pin-calibrated offset with the same pen and claw.

## Step 4 — Teach the canvas (real paper, freedrive + nudge)

Pass the **same** `tool_offset_xyz` the executor will use — the pin-calibrated value from Step 3.5
if you ran it. The corners are recorded in pen-tip coordinates, so **re-teach the canvas whenever
the tool offset changes**; a canvas taught against a stale offset is wrong.

Teach each corner at **just-touch** (spring at free length): the recorded point is the
free-length virtual tip, so any compression at record time pushes the taught plane that far
behind the paper. The current 1.0 mm drawing preload is applied in software by
`plane_bias_mm`. Do not increase it to hide direction-dependent tracking error.
Terminal 3 and 4:

```bash
ros2 run robross_painter teach_canvas.py --ros-args \
  -p tool_offset_xyz:="[0.0, -0.0595, 0.0514]" \
  -p plane_bias_mm:=1.0 \
  -p output_file:=$HOME/canvas_calibration.yaml

ros2 launch robross_painter teach_nudge.launch.py aubo_type:=$AUBO_TYPE \
  tool_offset_rpy:="[0.0, 0.0, 0.0]"      # launch (not run): supplies the
                                          # robot model; needs Terminal 2's move_group
```

Per corner: freedrive to hover a few mm out (freedrive breakaway force is too high for
accurate small motions), disable freedrive, reactivate `joint_trajectory_controller`, then
step in with `/teach_nudge/nudge_in` (drop to `nudge_step_mm 0.2` for the last mm) until the
pen body first visibly moves relative to the claw — stop there and record. Then `nudge_out`
clear, controller off, freedrive to the next corner (full loop: package README / PREFLIGHT
section 2). A record is rejected if the arm moved in the last second — wait, re-record.
All four corners are required and feed the least-squares plane fit (`save` still warns if
bottom-right sits > 2 mm from where the other three predict it). Then record ~5-9 interior
points the same way — spread across the paper (a rough 3×3: center, mid-edges, quarter
points). These fit a Z-correction surface recorded in the saved YAML as a flatness
diagnostic only — the executor does **not** apply it during motion
(`docs/aubo-painting-tracking-remediation-plan.md` Section 4 forbids position-dependent Z
compensation). The fit measures the reach-dependent, non-planar contact error that a flat
plane cannot represent, so a badly warped setup is caught at teach time:

```bash
ros2 service call /teach_canvas/record_top_left     std_srvs/srv/Trigger
ros2 service call /teach_canvas/record_top_right    std_srvs/srv/Trigger
ros2 service call /teach_canvas/record_bottom_left  std_srvs/srv/Trigger
ros2 service call /teach_canvas/record_bottom_right std_srvs/srv/Trigger
ros2 service call /teach_canvas/record_sample       std_srvs/srv/Trigger  # x5-9, interior
ros2 service call /teach_canvas/save                std_srvs/srv/Trigger
```

**Gate:** `save` must report ≈210 × 297 mm, an out-of-plane error after correction under
`flatness_warn_mm` (default 0.3 mm), and no bottom-right residual warning. `save` refuses
outright above `flatness_refuse_mm` (default 0.6 mm) — add interior samples or re-teach. Any
warning → re-teach; don't rationalize.

## Step 5 — Dry-run everything (`dry_run: true` in `~/hardware_a4.yaml`)

Full artwork, the test line, then the curve test card:

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=$AUBO_TYPE \
  calibration_file:=$HOME/hardware_a4.yaml \
  canvas_file:=$HOME/canvas_calibration.yaml \
  paths_file:=$ROBROSS_REPO/output/painting_paths.json
# then again with paths_file:=$ROBROSS_REPO/output/test_line_paths.json
# then again with paths_file:=$ROBROSS_REPO/output/curve_test_paths.json
```

**Gate:** all commands plan cleanly, arm never moves. Repeated
`Cartesian path only X% feasible` in one canvas region → try a different `tool_spin_deg` or move
the canvas; never lower `cartesian_jump_threshold`. A motion-guard rejection is a rejected plan,
not a parameter-tuning prompt.

## Step 5.5 - Screen the pushed Phase 2 ServoJ changes above the paper

Complete this section before any paper-contact run. It covers the pushed Aubo driver changes
that make ServoJ `t` configurable, publish timing diagnostics, add the 125 Hz trial profile,
reject malformed `servoj_time` values, and add offline timing/tracking analysis in
`robross_painter`.

The objective is an A/B comparison with exactly one timing variable changed:

| Trial | Controller file | Update rate | Required `servoj_time` |
| --- | --- | ---: | ---: |
| A | `aubo_controllers_125hz.yaml` | 125 Hz | `0.008` s |
| B | `aubo_controllers.yaml` | 200 Hz | `0.005` s |

Do not combine both trials in one driver session. Stop the executor, MoveIt, and driver after
Trial A, then start a new bag and a new stack for Trial B. Keep the robot pose, path files,
canvas, tool, velocity scaling, and acceleration scaling unchanged.

Before starting the checks below, stop the stack used for Step 5 and confirm no
`controller_manager`, MoveIt, or painting executor process from that stack remains. The bag
recorder for each timing trial must be running before its fresh driver starts.

### 5.5.1 Build, test, and record the exact revisions

From the workspace root:

```bash
export PHASE2_SESSION=$HOME/robross_phase2_$(date +%Y%m%d_%H%M%S)
mkdir "$PHASE2_SESSION"
{
  git -C src/RobRoss status --short --branch
  git -C src/RobRoss rev-parse HEAD
  git -C src/aubo_ros2_driver status --short --branch
  git -C src/aubo_ros2_driver rev-parse HEAD
  git -C src/aubo_ros2_driver submodule status
  git -C src/aubo_ros2_driver/aubo_description status --short --branch
  git -C src/aubo_ros2_driver/aubo_description rev-parse HEAD
} | tee $PHASE2_SESSION/source_revisions.txt
cp $HOME/canvas_calibration.yaml $PHASE2_SESSION/contact_canvas_source.yaml
cp $HOME/hardware_a4.yaml $PHASE2_SESSION/contact_hardware_source.yaml

colcon build --packages-select aubo_description aubo_ros2_driver robross_painter
source install/setup.bash
colcon test --packages-select aubo_ros2_driver robross_painter
colcon test-result --verbose
```

**Gate:** the build succeeds, `colcon test-result --verbose` reports no failures, both parent
repositories and the description repository are clean, and `git submodule status` has no leading
`+` or `-`. Keep this terminal open and export the same `PHASE2_SESSION` value in every trial
terminal. Do not compare bags produced by different source trees, including uncommitted changes.

### 5.5.2 One-time malformed `servoj_time` startup checks

Run these checks with the loopback address, not the robot address. This verifies that malformed
values fail during hardware-interface initialization before any robot connection or ServoJ
stream can begin:

```bash
timeout 15s ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_i5 robot_ip:=127.0.0.1 use_fake_hardware:=false \
  servoj_time:=nan 2>&1 | tee $PHASE2_SESSION/servoj_time_nan.log
grep -F "is not a finite number" $PHASE2_SESSION/servoj_time_nan.log

timeout 15s ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_i5 robot_ip:=127.0.0.1 use_fake_hardware:=false \
  servoj_time:=0.005junk 2>&1 | tee $PHASE2_SESSION/servoj_time_trailing.log
grep -F "has trailing characters after the number" \
  $PHASE2_SESSION/servoj_time_trailing.log
```

The launch process may remain alive until `timeout` stops it after the component rejects the
hardware configuration. The required result is the matching fatal message and no attempt to
connect to a real controller.

**Gate:** both `grep` commands find their expected rejection. Do not continue if either value is
accepted or reaches ServoJ setup.

### 5.5.3 Confirm the diagnostic fixtures

The Phase 2 timing comparison runs the direction fixture first and the sine fixture second so
the primary timing evidence is collected before additional paths can change controller or robot
state. The reversal and curve fixtures follow:

```bash
for fixture in \
  arm_tracking_direction_test_paths.json \
  sine_test_paths.json \
  arm_tracking_reversal_test_paths.json \
  arm_tracking_curve_test_paths.json; do
  test -f "$ROBROSS_REPO/output/$fixture" || echo "MISSING: $fixture"
done
```

Validate all four files from the RobRoss repository root:

```bash
python3 - <<'PY'
import json
from pathlib import Path

from Image_Process.mondrian.path_validation import validate_painting_paths

names = [
    "arm_tracking_direction_test_paths.json",
    "sine_test_paths.json",
    "arm_tracking_reversal_test_paths.json",
    "arm_tracking_curve_test_paths.json",
]
for name in names:
    path = Path("output") / name
    result = validate_painting_paths(json.loads(path.read_text()))
    print(f"{path}: {'PASS' if result['passed'] else 'FAIL'}")
    for error in result["errors"]:
        print(f"  ERROR: {error}")
    if not result["passed"]:
        raise SystemExit(1)
PY
sha256sum \
  $ROBROSS_REPO/output/arm_tracking_direction_test_paths.json \
  $ROBROSS_REPO/output/sine_test_paths.json \
  $ROBROSS_REPO/output/arm_tracking_reversal_test_paths.json \
  $ROBROSS_REPO/output/arm_tracking_curve_test_paths.json | \
  tee $PHASE2_SESSION/fixture_sha256.txt
```

**Gate:** all four files exist, pass validation, and retain the recorded hashes for Trial A and
Trial B. If they are missing, stop the formal timing-fixture run. The checked-in
`output/curve_test_paths.json` may be used for an above-paper instrumentation smoke test, but it
does not replace the direction, reversal, curve, and sine acceptance fixtures.

### 5.5.4 Create dedicated hover-only canvas and executor files

`dry_run: true` plans without moving and therefore cannot measure physical tracking. For this
test the arm must move with `dry_run: false`, but the commanded drawing plane must be safely in
front of the physical paper. Never use the contact canvas for this test.

The following creates `$HOME/canvas_hover_10mm.yaml` by shifting the taught canvas origin 10 mm
opposite canvas +Z. Canvas +Z points into the paper, so this moves the entire path out toward the
robot while preserving canvas X/Y and orientation:

```bash
CANVAS_IN=$HOME/canvas_calibration.yaml \
CANVAS_OUT=$HOME/canvas_hover_10mm.yaml \
HOVER_OFFSET_M=0.010 \
python3 - <<'PY'
import math
import os
from pathlib import Path

import yaml

source = Path(os.environ["CANVAS_IN"])
target = Path(os.environ["CANVAS_OUT"])
offset = float(os.environ["HOVER_OFFSET_M"])
data = yaml.safe_load(source.read_text())
params = data["painting_executor"]["ros__parameters"]
origin = [float(v) for v in params["canvas_origin_xyz"]]
quat = [float(v) for v in params["canvas_quat_xyzw"]]
norm = math.sqrt(sum(v * v for v in quat))
if norm <= 0.0:
    raise SystemExit("invalid zero-length canvas quaternion")
qx, qy, qz, qw = (v / norm for v in quat)
canvas_z = [
    2.0 * (qx * qz + qy * qw),
    2.0 * (qy * qz - qx * qw),
    1.0 - 2.0 * (qx * qx + qy * qy),
]
hover_origin = [origin[i] - offset * canvas_z[i] for i in range(3)]
params["canvas_origin_xyz"] = [round(v, 9) for v in hover_origin]
target.write_text(
    "# HOVER ONLY - 10 mm origin shift, never use for paper contact.\n"
    + yaml.safe_dump(data, sort_keys=False)
)
print(f"canvas +Z: {canvas_z}")
print(f"contact origin: {origin}")
print(f"hover origin:   {hover_origin}")
print(f"wrote {target}")
PY
```

Create a separate executor profile for physical hover motion. Do not change the reviewed contact
profile:

```bash
cp $HOME/hardware_a4.yaml $HOME/hardware_hover_a4.yaml
HOVER_CONFIG=$HOME/hardware_hover_a4.yaml python3 - <<'PY'
import os
from pathlib import Path

import yaml

path = Path(os.environ["HOVER_CONFIG"])
data = yaml.safe_load(path.read_text())
params = data["painting_executor"]["ros__parameters"]
params["dry_run"] = False
path.write_text(
    "# HOVER ONLY - dry_run is false; pair only with the shifted hover canvas.\n"
    + yaml.safe_dump(data, sort_keys=False)
)
print(f"dry_run={params['dry_run']}; wrote hover-only profile {path}")
PY
```

Open both generated files and verify the only intentional behavioral differences are the 10 mm
outward origin shift, durable `HOVER ONLY` warning, and `dry_run: false`. Because the taught
contact origin is already 1 mm into the paper, this shift produces approximately 9 mm of
physical paper clearance, not 10 mm. Keep velocity and acceleration scaling at `0.1`. RViz moves
the virtual backing with the hover canvas and therefore does not preserve the original physical
paper plane; verify stationary clearance physically before running a path.

**Gate:** the stationary pen remains at least several millimeters clear of the paper and backing,
the shifted plane stays collision-free, and the operator has the e-stop. Label both generated
files `HOVER ONLY`; never pass `canvas_hover_10mm.yaml` to a contact run.

Before real motion, execute every diagnostic fixture with fake hardware. Terminal 1:

```bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=$AUBO_TYPE use_fake_hardware:=true
```

Terminal 2:

```bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=$AUBO_TYPE
```

Terminal 3, repeat for all four fixtures in the Phase 2 order above:

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=$AUBO_TYPE \
  calibration_file:=$HOME/hardware_hover_a4.yaml \
  canvas_file:=$HOME/canvas_hover_10mm.yaml \
  paths_file:=$ROBROSS_REPO/output/arm_tracking_direction_test_paths.json
```

Inspect each complete trajectory, claw/canvas backing geometry, interpolation validation, and
elbow/guard behavior in RViz. **Gate:** all four fixtures complete in fake hardware with the
calibrated geometry and no collision, orientation, interpolation, or motion-guard regression.
Stop the fake driver and MoveIt before starting Trial A.

### 5.5.5 Trial A: 125 Hz controller with ServoJ `t=0.008 s`

Start the bag recorder before the driver so the bag captures the one-time `servoj_config` log.
In a recording terminal:

With the real driver stopped, return the arm from the pendant to one recorded, collision-free
elbow-up starting pose. Use this same pose for both trials.

```bash
export TRIAL_DIR=$PHASE2_SESSION/trial_a_125hz_008s
test ! -e "$TRIAL_DIR" || { echo "Choose a new Trial A bag path"; exit 1; }
ros2 bag record -o "$TRIAL_DIR" \
  /joint_trajectory_controller/controller_state \
  /joint_states \
  /robot_description \
  /rosout \
  /tf \
  /tf_static \
  /robross_markers
```

With the recorder running, start a fresh driver in Terminal 1:

```bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=$AUBO_TYPE robot_ip:=<ROBOT_IP> use_fake_hardware:=false \
  controllers_file:=aubo_controllers_125hz.yaml servoj_time:=0.008
```

Start MoveIt in Terminal 2 as in Step 3. In another terminal, record the effective controller
rate and preserve the exact inputs:

```bash
ros2 param get /controller_manager update_rate | \
  tee $PHASE2_SESSION/trial_a_update_rate.txt
ros2 param dump /controller_manager > \
  $PHASE2_SESSION/trial_a_controller_manager.yaml
ros2 param dump /joint_trajectory_controller > \
  $PHASE2_SESSION/trial_a_joint_trajectory_controller.yaml
ros2 topic echo /joint_states --once > \
  $PHASE2_SESSION/trial_a_start_joint_states.yaml
cp $HOME/canvas_hover_10mm.yaml $PHASE2_SESSION/trial_a_canvas.yaml
cp $HOME/hardware_hover_a4.yaml $PHASE2_SESSION/trial_a_hardware.yaml
```

Run each fixture through the hover plane, in order:

```bash
export TRIAL_NAME=trial_a
set -o pipefail
```

Use this same loop for each trial after setting its `TRIAL_NAME`:

```bash
for fixture in \
  arm_tracking_direction_test_paths.json \
  sine_test_paths.json \
  arm_tracking_reversal_test_paths.json \
  arm_tracking_curve_test_paths.json; do
  ros2 launch robross_painter paint.launch.py \
    aubo_type:=$AUBO_TYPE \
    calibration_file:=$HOME/hardware_hover_a4.yaml \
    canvas_file:=$HOME/canvas_hover_10mm.yaml \
    paths_file:=$ROBROSS_REPO/output/$fixture 2>&1 | \
    tee $PHASE2_SESSION/${TRIAL_NAME}_${fixture%.json}.log || break
done
```

While each executor is active, use another terminal to preserve its effective layered
parameters. Export the same trial name in that terminal and replace `<fixture>` with
`direction`, `sine`, `reversal`, or `curve`:

```bash
export TRIAL_NAME=trial_a
ros2 param dump /painting_executor > \
  $PHASE2_SESSION/${TRIAL_NAME}_<fixture>_painting_executor.yaml
```

If the node exits before the dump, invalidate the complete trial. Stop the bag and stack, return
to the recorded start pose with the driver stopped, and restart the trial under a new bag path.
Do not append a duplicate fixture to the active A/B bag; an input-file copy is not a substitute
for the effective runtime parameters.

Stop immediately for queue-full warnings, nonzero `servoj_rc`, a timing fault, visible wrist
oscillation toward the paper, unexpected path geometry, or loss of hover clearance. After all
fixtures finish, stop the bag recorder cleanly with Ctrl-C, then stop MoveIt and the driver.

### 5.5.6 Trial B: 200 Hz controller with ServoJ `t=0.005 s`

Repeat the complete startup and recording sequence with a new bag. With the driver stopped,
return the arm to the same recorded starting pose used for Trial A, then start the recorder.
After the Trial B driver publishes `/joint_states`, compare the measured start against
`trial_a_start_joint_states.yaml`; any joint differing by more than `0.5 deg` must be corrected
before fixture execution.

```bash
export TRIAL_DIR=$PHASE2_SESSION/trial_b_200hz_005s
test ! -e "$TRIAL_DIR" || { echo "Choose a new Trial B bag path"; exit 1; }
ros2 bag record -o "$TRIAL_DIR" \
  /joint_trajectory_controller/controller_state \
  /joint_states \
  /robot_description \
  /rosout \
  /tf \
  /tf_static \
  /robross_markers
```

If an existing bag requires a different output name, use that same replacement path in the
analysis commands below; never record over or delete a prior comparison bag.

Start a fresh driver with the matched 200 Hz pair:

```bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=$AUBO_TYPE robot_ip:=<ROBOT_IP> use_fake_hardware:=false \
  controllers_file:=aubo_controllers.yaml servoj_time:=0.005
```

Start MoveIt, then preserve the effective rate and exact input files:

```bash
ros2 param get /controller_manager update_rate | \
  tee $PHASE2_SESSION/trial_b_update_rate.txt
ros2 param dump /controller_manager > \
  $PHASE2_SESSION/trial_b_controller_manager.yaml
ros2 param dump /joint_trajectory_controller > \
  $PHASE2_SESSION/trial_b_joint_trajectory_controller.yaml
ros2 topic echo /joint_states --once > \
  $PHASE2_SESSION/trial_b_start_joint_states.yaml
cp $HOME/canvas_hover_10mm.yaml $PHASE2_SESSION/trial_b_canvas.yaml
cp $HOME/hardware_hover_a4.yaml $PHASE2_SESSION/trial_b_hardware.yaml
```

Compare the two recorded starting poses. The values in `/joint_states` are radians:

```bash
TRIAL_A=$PHASE2_SESSION/trial_a_start_joint_states.yaml \
TRIAL_B=$PHASE2_SESSION/trial_b_start_joint_states.yaml \
python3 - <<'PY'
import math
import os
from pathlib import Path

import yaml

def load(path):
    doc = next(yaml.safe_load_all(Path(path).read_text()))
    return dict(zip(doc["name"], doc["position"]))

a = load(os.environ["TRIAL_A"])
b = load(os.environ["TRIAL_B"])
expected = {
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
}
if set(a) != expected or set(b) != expected:
    raise SystemExit(
        f"start-pose joint set mismatch: trial_a={sorted(a)}, trial_b={sorted(b)}"
    )
worst_name = None
worst_deg = 0.0
for name in sorted(expected):
    delta_deg = abs(math.degrees(float(b[name]) - float(a[name])))
    print(f"{name}: {delta_deg:.3f} deg")
    if delta_deg > worst_deg:
        worst_name, worst_deg = name, delta_deg
if worst_name is None or worst_deg > 0.5:
    raise SystemExit(
        f"start-pose gate failed: {worst_name} differs by {worst_deg:.3f} deg"
    )
PY
```

If this gate fails, do not correct the pose while the ROS position controller is active. Stop
the Trial B bag, MoveIt, and driver; reposition from the pendant with the driver stopped; and
restart Trial B using a new bag path.

Export `TRIAL_NAME=trial_b` in both the executor and parameter-dump terminals, execute the same
four-fixture loop from Trial A, and dump each `/painting_executor` parameter set using that trial
name. Stop the recorder, MoveIt, and driver when finished. Do not adjust velocity scaling, path
geometry, tool pose, canvas pose, or any ServoJ parameter between trials.

### 5.5.7 Analyze each bag offline

Run analysis after the hardware stack is stopped. Use the same hover canvas and executor profile
that produced the bag, and pass the plane bias recorded in the original canvas file (`1.0 mm` for
the current procedure):

```bash
set -o pipefail
grep -F "# plane_bias_mm: 1.0" $PHASE2_SESSION/contact_canvas_source.yaml || {
  echo "Plane-bias evidence is missing or does not match 1.0 mm"
  exit 1
}

ros2 run robross_painter analyze_tracking_bag.py \
  $PHASE2_SESSION/trial_a_125hz_008s \
  --canvas-file $PHASE2_SESSION/trial_a_canvas.yaml \
  --calibration-file $PHASE2_SESSION/trial_a_hardware.yaml \
  --plane-bias-mm 1.0 \
  --csv $PHASE2_SESSION/trial_a_tracking.csv \
  --servoj-csv $PHASE2_SESSION/trial_a_servoj.csv | \
  tee $PHASE2_SESSION/trial_a_summary.md

ros2 run robross_painter analyze_tracking_bag.py \
  $PHASE2_SESSION/trial_b_200hz_005s \
  --canvas-file $PHASE2_SESSION/trial_b_canvas.yaml \
  --calibration-file $PHASE2_SESSION/trial_b_hardware.yaml \
  --plane-bias-mm 1.0 \
  --csv $PHASE2_SESSION/trial_b_tracking.csv \
  --servoj-csv $PHASE2_SESSION/trial_b_servoj.csv | \
  tee $PHASE2_SESSION/trial_b_summary.md
```

For Trial A, the summary must show `config: t=0.008` and an effective configured rate of 125 Hz.
For Trial B, it must show `config: t=0.005` and 200 Hz. If the config line is absent, the bag was
started too late; repeat the trial instead of interpreting the timing gate.

Record the following comparison in the session notes:

| Metric | Trial A: 125 Hz / 8 ms | Trial B: 200 Hz / 5 ms |
| --- | ---: | ---: |
| Effective update rate and percent configured | | |
| Period mean / p95 / p99 / maximum | | |
| ServoJ RPC mean / maximum | | |
| Queue-full events and retries | | |
| Non-OK return codes and exceptions | | |
| Median / p95 phase delay | | |
| Worst actual canvas-normal error | | |
| Sine normal peak-to-peak | | |
| Visible movement-synchronized wrist motion | | |

### 5.5.8 Screening gates and interpretation rules

Apply these gates when selecting a timing pair for further engineering work:

- Actual update rate is at least 95 percent of the configured rate.
- No queue-full event, unexplained ServoJ return code, exception, or timing fault occurred.
- Oscillatory fixtures report delay values, with median below 30 ms and p95 below 50 ms.
- Actual canvas-normal tracking stays within `+/-0.25 mm` during hover motion.
- The executor's Cartesian validation log confirms that the controller reference remains within
  the Phase 1 `+/-0.20 mm` normal limit.
- No visible movement-synchronized wrist oscillation remains.
- Direction, reversal, curve, and sine paths all complete above the paper without a safety abort.

Preserve and inspect the executor evidence with:

```bash
grep -h "Cartesian FK error after retiming" $PHASE2_SESSION/trial_*.log
```

Every reported normal into/out-of-paper magnitude must be no greater than `0.20 mm`. Missing
lines are incomplete evidence, not a pass.

Treat a missing ServoJ configuration, missing report window, malformed diagnostics, or
`delay n/a` for the sine fixture as **INCOMPLETE**, even if the current analyzer prints `PASS`.
The pushed analyzer estimates phase delay from controller-state publication and the pushed
driver currently emits windowed timing statistics rather than every timestamped ServoJ command.
These results are suitable for A/B screening, but they do not by themselves satisfy the
remediation plan's raw per-call telemetry requirement.

If neither timing pair satisfies every measurable gate, stop. Do not proceed to contact, do not
increase plane bias, do not add direction-dependent Z compensation, and do not tune ServoJ
lookahead or gain. Investigate the RPC streaming path first. If one pair is clearly acceptable,
record that complete launch pair as a candidate for the next implementation step; controller
rate and ServoJ `t` must always be reviewed together. A candidate does not authorize contact.
Paper contact remains blocked until the driver records every timestamped ServoJ command/call
interval, the analyzer computes acceptance-grade temporal delay distributions from that data,
and all formal Phase 2 gates pass.

## Step 6 — First contact: the 50 mm line

**Current gate:** do not run this step on the pushed Phase 2 revisions described in Step 5.5.
Their diagnostics support A/B screening but do not yet provide the required per-call telemetry.
Begin contact only after a later reviewed implementation closes that gap and the formal Phase 2
gate passes. Then edit `~/hardware_a4.yaml`: `dry_run: false` (keep scaling at 0.1), return to the
original `~/canvas_calibration.yaml` (never the hover canvas), clear the arm's entire reach
sphere, and keep one hand on the e-stop:

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=$AUBO_TYPE \
  calibration_file:=$HOME/hardware_a4.yaml \
  canvas_file:=$HOME/canvas_calibration.yaml \
  paths_file:=$ROBROSS_REPO/output/test_line_paths.json
```

**Gate:** ~50 mm horizontal line at (80,140)→(130,140), uniform darkness (fading toward one side
means the taught plane is tilted → re-teach), pen never bottoms out audibly, paper undamaged.
Compare against `output/test_line_preview.svg`.

## Step 7 — Curves and corners

Keep `dry_run: false` and scaling at 0.1. Run this only after the 50 mm line passes:

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=$AUBO_TYPE \
  calibration_file:=$HOME/hardware_a4.yaml \
  canvas_file:=$HOME/canvas_calibration.yaml \
  paths_file:=$ROBROSS_REPO/output/curve_test_paths.json
```

**Gate:** four separate shapes are drawn with a lift between them; the circle closes, the S-curve
and squiggle are smooth, and the right-angle and acute corners remain distinct. Compare against
`output/curve_test_preview.svg`. Stop if the pen leaves the paper bounds, chatters, digs in, or
takes an unexpected shortcut between shapes.

## Step 8 — Full artwork

Same command with `paths_file:=$ROBROSS_REPO/output/painting_paths.json`; compare the result
against `output/path_preview.svg`. Raise `velocity_scaling` only after motion is trusted.

## Session rules (PREFLIGHT §5)

- **Stack restart ⇒ painting restart.** If the driver or move_group restarts mid-run, the
  planning scene is empty — never "resume" a painting, rerun it.
- Start the arm inside the elbow-up band (freedrive/pendant) or the executor aborts before moving.
- Abort with pen down = straight lift only; if the lift fails, jog the pen clear manually before
  doing anything else.
- Never edit safety params (`cartesian_jump_threshold`, backing/claw settings, guard limits)
  mid-session to get past a failure — a failure means the motion could not be verified safe.
