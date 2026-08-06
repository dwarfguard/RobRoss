# Hardware Run Guide: Aubo i5 over Ethernet

Step-by-step commands for qualifying the driver on a real arm and progressing from stationary
checks to hover motion, first contact, and the full A4 artwork. This is the exact-command
walkthrough and the authoritative per-session hardware procedure. Use the
[current status](aubo-painting-current-status-2026-07-31.md) for engineering decisions and
evidence interpretation.

## Readiness status (as of 2026-07-31)

**Demonstrated baseline:**

- The calibrated `aubo_i5_calibrated` model activates and executes the reviewed paths.
- The selected `125 Hz / ServoJ t=0.008 s` pair completed stationary lifecycle checks, three
  deliberate hover repetitions, and a supervised contact artwork without queue-full events,
  non-OK ServoJ returns, telemetry drops, or timing faults.
- The supervised contact bag contains all 1,466 commands and 366 paint strokes; the operator
  judged the drawing acceptable.
- The taught A4 canvas, measured tool offset, claw collision geometry, and spring preload were
  exercised on hardware.
- Full results and interpretation rules are in the
  [current status](aubo-painting-current-status-2026-07-31.md).

**Required before each new or changed setup:**

1. Preserve the exact source, path, calibration, canvas, controller, and profile inputs.
2. Re-run the dry-run and Step 5.5 hover qualification after a relevant source, controller,
   calibration, tool, or motion-profile change.
3. Keep supervised contact under immediate e-stop control. Unattended contact remains
   unapproved until endpoint settling and continuous contact guarding are implemented.

## Step 0 — Network (Ethernet direct)

1. Cable the PC to the Aubo control box LAN port. Read the controller IP from the pendant.
2. Put the PC on the same subnet (e.g. controller `192.168.127.128` → PC `192.168.127.100/24`,
   via the desktop network settings or `nmcli`).
3. Read and verify the address with this complete block:

```bash
read -r -p "Robot controller IP: " ROBOT_IP
export ROBOT_IP
ping -c 4 "$ROBOT_IP"
```

## Step 1 — Session environment

Run once in the first terminal. This creates one reusable environment file so every terminal
uses the same robot, source tree, and evidence directory:

```bash
export PATH="/usr/bin:$PATH"
cd ~/robross_aubo_ws
source install/setup.bash
: "${ROBOT_IP:?Run Step 0 in this terminal before creating the session}"
export ROBROSS_REPO=$PWD/src/RobRoss
export AUBO_TYPE=${AUBO_TYPE:-aubo_i5_calibrated}
export HARDWARE_SESSION=$HOME/robross_hardware_$(date +%Y%m%d_%H%M%S)
mkdir -p "$HARDWARE_SESSION"
cat > "$HOME/robross_hardware_session.env" <<EOF
export PATH="/usr/bin:\$PATH"
export ROBOT_IP="$ROBOT_IP"
export AUBO_TYPE="$AUBO_TYPE"
export HARDWARE_SESSION="$HARDWARE_SESSION"
export ROBROSS_REPO="$ROBROSS_REPO"
EOF
printf 'Session evidence: %s\n' "$HARDWARE_SESSION"
```

Run this complete block in every additional terminal:

```bash
cd ~/robross_aubo_ws
source install/setup.bash
source "$HOME/robross_hardware_session.env"
```

## Step 2 — (Recommended, once) URDF calibration from the controller

```bash
python3 -m pip install --user pyaubo-sdk==0.24.1
python3 -c "import numpy, pyaubo_sdk"
python3 src/aubo_ros2_driver/aubo_description/scripts/calibrate_urdf_dh.py \
  --robot-model aubo_i5 --robot-ip "$ROBOT_IP"
colcon build --packages-select aubo_description
source install/setup.bash
export AUBO_TYPE=aubo_i5_calibrated
printf 'export AUBO_TYPE=%q\n' "$AUBO_TYPE" >> \
  "$HOME/robross_hardware_session.env"
```

Keep `AUBO_TYPE=aubo_i5_calibrated` in every terminal after calibration. Using
`aubo_i5` would silently return to the stock model.

## Step 2.5 - Verify the physical model and hardware profile

Create a session copy without overwriting an existing measured profile:

```bash
test -f "$HOME/hardware_a4.yaml" || \
  cp "$ROBROSS_REPO/ros2/robross_painter/config/hardware_a4.yaml" \
  "$HOME/hardware_a4.yaml"
```

Review the complete file against the mounted hardware before launching:

- `~/hardware_a4.yaml` is the real-arm profile, not an RViz profile.
- `tool_offset_xyz` and `tool_offset_rpy` match the mounted claw and pen. Recalibrate after any
  pen or claw change, then re-teach the canvas.
- `claw_collision_size_xyz` generously encloses the real claw and the pen tip protrudes beyond
  it. If collision validation rejects the geometry, measure again; do not shrink the box merely
  to make a plan pass.
- `canvas_backing_enabled: true`, and the real backing surface extends beyond the paper by at
  least `canvas_backing_margin_m`. Clear clamps, frames, cables, table edges, people, and other
  objects that are not represented in the planning scene.
- If `ground_enabled: true`, `ground_z_m` matches the physical mounting surface.
- `cartesian_jump_threshold` is nonzero, the elbow and guarded-joint limits are unchanged from
  the reviewed profile, and velocity/acceleration scaling is appropriate for the staged run.
- `controller_sample_dt: 0.008` matches the selected 125 Hz controller period.
- Keep the contact profile at `dry_run: true` until Step 6. Step 5.5 creates a separate,
  clearly labeled hover-only copy for above-paper motion.

## Step 3 — Bring up the real-arm stack

Terminal 1 (driver, real hardware):
```bash
export AUBO_SERVOJ_TELEMETRY_CSV=$HARDWARE_SESSION/stationary_servoj.csv
test ! -e "$AUBO_SERVOJ_TELEMETRY_CSV" || {
  echo "Choose a new HARDWARE_SESSION; telemetry output already exists"
  exit 1
}
test ! -e "$AUBO_SERVOJ_TELEMETRY_CSV.partial" || {
  echo "Choose a new HARDWARE_SESSION; partial telemetry output already exists"
  exit 1
}
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=$AUBO_TYPE robot_ip:=$ROBOT_IP use_fake_hardware:=false \
  controllers_file:=aubo_controllers_125hz.yaml servoj_time:=0.008
```

The selected hardware profile is the matched pair `125 Hz / 0.008 s`, and it is now the driver
default: a bare `aubo_control.launch.py robot_ip:=<IP>` comes up on it, so the explicit
`controllers_file:=aubo_controllers_125hz.yaml servoj_time:=0.008` args above are optional (they
just restate the defaults). Do not change only one value. The `200 Hz / 0.005 s` pair is
disqualified — never a launch option — because valid stationary evidence showed approximately
20.4 percent queue-full drops; `aubo_controllers.yaml` is retained on disk for historical
diagnostics only. See the [current status](aubo-painting-current-status-2026-07-31.md).

The driver launch argument `rtde_state_max_age` defaults to `0.05` seconds (allowed range
`0.005`–`0.100`). If the source RTDE joint-state packet exceeds that age, the driver latches a
hardware fault, invalidates velocity feedback, and refuses further ServoJ writes until restart.
Keep it at or below the painter's `endpoint_settle_sample_max_age`.

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
read -r -p "tool_offset_xyz exactly as YAML ([x, y, z]): " TOOL_OFFSET_XYZ
read -r -p "tool_offset_rpy exactly as YAML ([r, p, y]): " TOOL_OFFSET_RPY
ros2 run robross_painter teach_canvas.py --ros-args \
  -p tool_offset_xyz:="$TOOL_OFFSET_XYZ" \
  -p plane_bias_mm:=1.0 \
  -p output_file:=$HOME/canvas_calibration.yaml

ros2 launch robross_painter teach_nudge.launch.py aubo_type:=$AUBO_TYPE \
  tool_offset_rpy:="$TOOL_OFFSET_RPY"      # launch (not run): supplies the
                                           # robot model; needs Terminal 2's move_group
```
```bash
   ros2 service call /teach_nudge/nudge_in std_srvs/srv/Trigger
   ros2 param set /teach_nudge nudge_step_mm 0.2   # finer steps for the last mm
```

Per corner: freedrive to hover a few mm out (freedrive breakaway force is too high for
accurate small motions), disable freedrive, reactivate `joint_trajectory_controller`, then
step in with `/teach_nudge/nudge_in` (drop to `nudge_step_mm 0.2` for the last mm) until the
pen body first visibly moves relative to the claw — stop there and record. Then `nudge_out`
clear, controller off, and freedrive to the next corner. A record is rejected if the arm moved
in the last second; wait and re-record rather than raising the tolerance.
All four corners are required and feed the least-squares plane fit (`save` still warns if
bottom-right sits > 2 mm from where the other three predict it). Then record ~5-9 interior
points the same way — spread across the paper (a rough 3×3: center, mid-edges, quarter
points). These fit a Z-correction surface recorded in the saved YAML as a flatness
diagnostic only — the executor does **not** apply it during motion. The current engineering
decision remains to diagnose tracking and mechanics rather than apply position-dependent Z
compensation; see the [current status](aubo-painting-current-status-2026-07-31.md). The fit measures the
reach-dependent, non-planar contact error that a flat
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

Run the full artwork, test line, and curve card with this complete loop:

```bash
set -euo pipefail
for path_file in \
  painting_paths.json \
  test_line_paths.json \
  curve_test_paths.json; do
  ros2 launch robross_painter paint.launch.py \
    aubo_type:=$AUBO_TYPE \
    calibration_file:=$HOME/hardware_a4.yaml \
    canvas_file:=$HOME/canvas_calibration.yaml \
    paths_file:=$ROBROSS_REPO/output/$path_file
done
```

**Gate:** all commands plan cleanly, arm never moves. Repeated
`Cartesian path only X% feasible` in one canvas region → try a different `tool_spin_deg` or move
the canvas; never lower `cartesian_jump_threshold`. A motion-guard rejection is a rejected plan,
not a parameter-tuning prompt.

## Step 5.5 - Qualify the reviewed ServoJ driver above the paper

Complete this section before the first paper-contact run on a new setup and repeat it after a
driver, controller, calibration, tool, or relevant motion-profile change. The July 31 baseline is
recorded in the [current status](aubo-painting-current-status-2026-07-31.md).

### 5.5.0 Mandatory stationary lifecycle and telemetry qualification

Keep the arm stationary, clear its reach sphere, and keep the position controller unloaded.
Terminal 1 must still be running the Step 3 driver with
`AUBO_SERVOJ_TELEMETRY_CSV=$HARDWARE_SESSION/stationary_servoj.csv`.

In Terminal 3, record the initial states, deactivate both controllers, and deactivate hardware:

```bash
set -euo pipefail
ros2 control list_hardware_components | tee "$HARDWARE_SESSION/hardware_before.txt"
ros2 control list_controllers | tee "$HARDWARE_SESSION/controllers_before.txt"
ros2 param get /controller_manager use_sim_time | \
  tee "$HARDWARE_SESSION/use_sim_time.txt"
grep -F "Boolean value is: False" "$HARDWARE_SESSION/use_sim_time.txt"
ros2 control switch_controllers --strict \
  --deactivate joint_trajectory_controller joint_state_broadcaster
ros2 control set_hardware_component_state auboHardwareInterface inactive
test -f "$HARDWARE_SESSION/stationary_servoj.csv"
test ! -e "$HARDWARE_SESSION/stationary_servoj.csv.partial"
tail -n 1 "$HARDWARE_SESSION/stationary_servoj.csv" | \
  tee "$HARDWARE_SESSION/first_activation_footer.txt"
grep -F "status=complete" "$HARDWARE_SESSION/first_activation_footer.txt"
cp "$HARDWARE_SESSION/stationary_servoj.csv" \
  "$HARDWARE_SESSION/stationary_servoj_first_activation.csv"
```

Reactivate in the same process, run stationary for two report windows, and deactivate again:

```bash
set -euo pipefail
ros2 control set_hardware_component_state auboHardwareInterface active
ros2 control switch_controllers --strict \
  --activate joint_state_broadcaster joint_trajectory_controller
sleep 5
ros2 control switch_controllers --strict \
  --deactivate joint_trajectory_controller joint_state_broadcaster
ros2 control set_hardware_component_state auboHardwareInterface inactive
test -f "$HARDWARE_SESSION/stationary_servoj.csv"
test ! -e "$HARDWARE_SESSION/stationary_servoj.csv.partial"
tail -n 1 "$HARDWARE_SESSION/stationary_servoj.csv" | \
  tee "$HARDWARE_SESSION/second_activation_footer.txt"
grep -F "status=complete" "$HARDWARE_SESSION/second_activation_footer.txt"
cp "$HARDWARE_SESSION/stationary_servoj.csv" \
  "$HARDWARE_SESSION/stationary_servoj_second_activation.csv"
ros2 control set_hardware_component_state auboHardwareInterface active
ros2 control switch_controllers --strict \
  --activate joint_state_broadcaster joint_trajectory_controller
ros2 topic echo /joint_states --once --no-lost-messages | \
  tee "$HARDWARE_SESSION/joint_states_after_reactivation.yaml"
```

Inspect both complete CSVs with a copy-pasteable parser:

```bash
SESSION="$HARDWARE_SESSION" python3 - <<'PY'
import csv
import os
from pathlib import Path

session = Path(os.environ["SESSION"])
for name in (
    "stationary_servoj_first_activation.csv",
    "stationary_servoj_second_activation.csv",
):
    path = session / name
    lines = path.read_text().splitlines()
    footer = lines[-1]
    rows = list(csv.DictReader(line for line in lines if not line.startswith("#")))
    if not rows:
        raise SystemExit(f"{path}: no ServoJ rows recorded")
    seq = [int(row["seq"]) for row in rows]
    if "status=complete" not in footer or "dropped=0" not in footer:
        raise SystemExit(f"{path}: invalid footer: {footer}")
    if seq and seq != list(range(seq[0], seq[0] + len(seq))):
        raise SystemExit(f"{path}: sequence gap")
    call_ros = [float(row["t_call_ros_s"]) for row in rows]
    cycle_ros = [float(row["t_ros_s"]) for row in rows]
    if any(value <= 0.0 for value in call_ros):
        raise SystemExit(f"{path}: invalid call timestamp")
    max_clock_delta = max(abs(call - cycle) for call, cycle in zip(call_ros, cycle_ros))
    if max_clock_delta > 1.0:
        raise SystemExit(f"{path}: ROS/system clock mismatch {max_clock_delta:.6f} s")
    print(
        f"PASS {path}: rows={len(rows)} max_clock_delta_s={max_clock_delta:.6f} "
        f"footer={footer}"
    )
PY
```

**Gate:** both activations succeed in one driver process, each deactivation disables Servo mode,
both files finalize with zero drops and contiguous sequence numbers, no `.partial` remains, and
live joint states resume. An unexpected activation failure still ends the qualification run;
preserve logs and restart the stack before any motion.

The stationary qualification establishes the selected timing pair:

| Status | Controller file | Update rate | Required `servoj_time` |
| --- | --- | ---: | ---: |
| Default (125 Hz / 0.008 s) | `aubo_controllers_125hz.yaml` | 125 Hz | `0.008` s |
| Disqualified | `aubo_controllers.yaml` | 200 Hz | `0.005` s |

The 200 Hz pair is disqualified and does not need to be rerun for routine qualification. Before the hover checks
below, stop the stack used for Step 5 and confirm no `controller_manager`, MoveIt, or painting
executor process remains. Start the bag before the fresh 125 Hz driver so it captures
`servoj_config`.

### 5.5.1 Build, test, and record the exact revisions

From the workspace root:

```bash
set -euo pipefail
export PHASE2_SESSION=$HARDWARE_SESSION
mkdir -p "$PHASE2_SESSION"
printf 'export PHASE2_SESSION=%q\n' "$PHASE2_SESSION" >> \
  "$HOME/robross_hardware_session.env"
{
  git -C src/RobRoss status --short --branch
  git -C src/RobRoss rev-parse HEAD
  git -C src/aubo_ros2_driver status --short --branch
  git -C src/aubo_ros2_driver rev-parse HEAD
  git -C src/aubo_ros2_driver submodule status
  git -C src/aubo_ros2_driver/aubo_description status --short --branch
  git -C src/aubo_ros2_driver/aubo_description rev-parse HEAD
} | tee $PHASE2_SESSION/source_revisions.txt
git -C src/RobRoss diff --binary HEAD > \
  $PHASE2_SESSION/robross_worktree.patch
git -C src/aubo_ros2_driver diff --binary HEAD > \
  $PHASE2_SESSION/aubo_driver_worktree.patch
git -C src/aubo_ros2_driver/aubo_description diff --binary HEAD > \
  $PHASE2_SESSION/aubo_description_worktree.patch
git -C src/RobRoss ls-files --others --exclude-standard -z | \
  tar -C src/RobRoss --null -czf \
  $PHASE2_SESSION/robross_untracked.tar.gz --files-from=-
git -C src/aubo_ros2_driver ls-files --others --exclude-standard -z | \
  tar -C src/aubo_ros2_driver --null -czf \
  $PHASE2_SESSION/aubo_driver_untracked.tar.gz --files-from=-
git -C src/aubo_ros2_driver/aubo_description \
  ls-files --others --exclude-standard -z | \
  tar -C src/aubo_ros2_driver/aubo_description --null -czf \
  $PHASE2_SESSION/aubo_description_untracked.tar.gz --files-from=-
sha256sum \
  $PHASE2_SESSION/*_worktree.patch \
  $PHASE2_SESSION/*_untracked.tar.gz | \
  tee $PHASE2_SESSION/source_evidence_sha256.txt
cp $HOME/canvas_calibration.yaml $PHASE2_SESSION/contact_canvas_source.yaml
cp $HOME/hardware_a4.yaml $PHASE2_SESSION/contact_hardware_source.yaml

colcon build --packages-select aubo_description aubo_ros2_driver robross_painter
source install/setup.bash
colcon test --packages-select aubo_ros2_driver robross_painter
colcon test-result --verbose
```

**Gate:** the build succeeds and `colcon test-result --verbose` reports no failures. A clean
worktree is preferred, but an intentional dirty build is valid evidence only when all patches and
input hashes are preserved before motion. `git submodule status` must have no leading `-`; a
leading `+` must be explained by the recorded description revision. Source the updated session
environment file in every additional terminal so `PHASE2_SESSION` is available.

### 5.5.2 One-time startup-failure checks

Run these checks with the loopback address, not the robot address. This verifies that malformed
values fail during hardware-interface initialization before any robot connection or ServoJ
stream can begin. It also exercises fail-fast telemetry initialization with a path that cannot
be opened:

```bash
timeout 15s ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_i5 robot_ip:=127.0.0.1 use_fake_hardware:=false \
  servoj_time:=nan 2>&1 | tee $PHASE2_SESSION/servoj_time_nan.log || true
grep -F "is not a finite number" $PHASE2_SESSION/servoj_time_nan.log

timeout 15s ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_i5 robot_ip:=127.0.0.1 use_fake_hardware:=false \
  servoj_time:=0.005junk 2>&1 | tee $PHASE2_SESSION/servoj_time_trailing.log || true
grep -F "has trailing characters after the number" \
  $PHASE2_SESSION/servoj_time_trailing.log

export AUBO_SERVOJ_TELEMETRY_CSV=/proc/aubo_servoj_forbidden.csv
timeout 15s ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_i5 robot_ip:=127.0.0.1 use_fake_hardware:=false \
  servoj_time:=0.005 2>&1 | tee $PHASE2_SESSION/telemetry_open_failure.log || true
grep -F "could not open" $PHASE2_SESSION/telemetry_open_failure.log
test ! -e /proc/aubo_servoj_forbidden.csv
test ! -e /proc/aubo_servoj_forbidden.csv.partial
unset AUBO_SERVOJ_TELEMETRY_CSV
```

The launch process may remain alive until `timeout` stops it after the component rejects the
hardware configuration. The required result is the matching fatal message and no attempt to
connect to a real controller.

**Gate:** all three `grep` commands find their expected rejection, no forbidden telemetry file is
created, and no check reaches a real controller or ServoJ setup.

### 5.5.3 Confirm the current diagnostic fixtures

The current hover suite runs direction, reversal, and alternating-curve motion. The former sine
fixture is obsolete and intentionally removed.

```bash
for fixture in \
  arm_tracking_direction_test_paths.json \
  arm_tracking_reversal_test_paths.json \
  arm_tracking_curve_test_paths.json; do
  test -f "$ROBROSS_REPO/output/$fixture" || echo "MISSING: $fixture"
done
```

Validate all three files from the RobRoss repository root:

```bash
python3 - <<'PY'
import json
from pathlib import Path

from Image_Process.mondrian.path_validation import validate_painting_paths

names = [
    "arm_tracking_direction_test_paths.json",
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
  $ROBROSS_REPO/output/arm_tracking_reversal_test_paths.json \
  $ROBROSS_REPO/output/arm_tracking_curve_test_paths.json | \
  tee $PHASE2_SESSION/fixture_sha256.txt
```

**Gate:** all three files exist, pass validation, and retain the recorded hashes. If any is
missing, stop the hover run. `output/curve_test_paths.json` remains a separate contact test card;
it does not replace the current tracking fixtures.

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
physical paper clearance, not 10 mm. Keep velocity and acceleration scaling unchanged from the
reviewed session profile. RViz moves the virtual backing with the hover canvas and therefore does
not preserve the original physical paper plane; verify stationary clearance physically before
running a path.

**Gate:** the stationary pen remains at least several millimeters clear of the paper and backing,
the shifted plane stays collision-free, and the operator has the e-stop. Label both generated
files `HOVER ONLY`; never pass `canvas_hover_10mm.yaml` to a contact run.

Before real motion, execute every diagnostic fixture with fake hardware. Terminal 1:

```bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=$AUBO_TYPE use_fake_hardware:=true \
  controllers_file:=aubo_controllers_125hz.yaml servoj_time:=0.008
```

Terminal 2:

```bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=$AUBO_TYPE
```

Terminal 3, repeat for all three fixtures in the order above:

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=$AUBO_TYPE \
  calibration_file:=$HOME/hardware_hover_a4.yaml \
  canvas_file:=$HOME/canvas_hover_10mm.yaml \
  paths_file:=$ROBROSS_REPO/output/arm_tracking_direction_test_paths.json
```

Inspect each complete trajectory, claw/canvas backing geometry, interpolation validation, and
elbow/guard behavior in RViz. **Gate:** all three fixtures complete in fake hardware with the
calibrated geometry and no collision, orientation, interpolation, or motion-guard regression.
Stop the fake driver and MoveIt before starting the recorded hover run.

### 5.5.5 Recorded hover run: 125 Hz controller with ServoJ `t=0.008 s`

Start the bag recorder before the driver so the bag captures the one-time `servoj_config` log.
In a recording terminal:

With the real driver stopped, return the arm from the pendant to one recorded, collision-free
elbow-up starting pose.

```bash
export TRIAL_DIR=$PHASE2_SESSION/hover_125hz_008s
test ! -e "$TRIAL_DIR" || { echo "Choose a new hover bag path"; exit 1; }
ros2 bag record -o "$TRIAL_DIR" \
  /joint_trajectory_controller/controller_state \
  /joint_states \
  /parameter_events \
  /robot_description \
  /rosout \
  /tf \
  /tf_static \
  /robross_markers
```

With the recorder running, start a fresh driver in Terminal 1:

```bash
export AUBO_SERVOJ_TELEMETRY_CSV=$PHASE2_SESSION/hover_servoj_calls.csv
test ! -e "$AUBO_SERVOJ_TELEMETRY_CSV" || {
  echo "Hover telemetry output already exists; choose a new session"
  exit 1
}
test ! -e "$AUBO_SERVOJ_TELEMETRY_CSV.partial" || {
  echo "Hover partial telemetry output already exists; choose a new session"
  exit 1
}
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=$AUBO_TYPE robot_ip:=$ROBOT_IP use_fake_hardware:=false \
  controllers_file:=aubo_controllers_125hz.yaml servoj_time:=0.008
```

Start MoveIt in Terminal 2 as in Step 3. In another terminal, record the effective controller
rate and preserve the exact inputs. `/parameter_events` captures executor parameter declarations
without racing the short-lived node with a manual parameter dump.

```bash
ros2 param get /controller_manager update_rate | \
  tee $PHASE2_SESSION/hover_update_rate.txt
ros2 param dump /controller_manager > \
  $PHASE2_SESSION/hover_controller_manager.yaml
ros2 param dump /joint_trajectory_controller > \
  $PHASE2_SESSION/hover_joint_trajectory_controller.yaml
ros2 topic echo /joint_states --once --no-lost-messages > \
  $PHASE2_SESSION/hover_start_joint_states.yaml
cp $HOME/canvas_hover_10mm.yaml $PHASE2_SESSION/hover_canvas.yaml
cp $HOME/hardware_hover_a4.yaml $PHASE2_SESSION/hover_hardware.yaml
cp src/aubo_ros2_driver/aubo_ros2_driver/config/aubo_controllers_125hz.yaml \
  $PHASE2_SESSION/hover_controller_profile.yaml
{
  printf 'aubo_type=%s\n' "$AUBO_TYPE"
  printf 'controllers_file=aubo_controllers_125hz.yaml\n'
  printf 'servoj_time=0.008\n'
  sha256sum \
    $PHASE2_SESSION/hover_canvas.yaml \
    $PHASE2_SESSION/hover_hardware.yaml \
    $PHASE2_SESSION/hover_controller_profile.yaml \
    $ROBROSS_REPO/output/arm_tracking_direction_test_paths.json \
    $ROBROSS_REPO/output/arm_tracking_reversal_test_paths.json \
    $ROBROSS_REPO/output/arm_tracking_curve_test_paths.json
} | tee $PHASE2_SESSION/hover_run_manifest.txt
```

Run each fixture through the hover plane in order. Repetitions are deliberate repeatability
evidence, not duplicate contamination. Set and preserve the intended repeat count:

```bash
export HOVER_REPEATS=${HOVER_REPEATS:-3}
set -euo pipefail
[[ "$HOVER_REPEATS" =~ ^[1-9][0-9]*$ ]] || {
  echo "HOVER_REPEATS must be a positive integer"
  exit 1
}
printf 'hover_repeats=%s\n' "$HOVER_REPEATS" | \
  tee -a $PHASE2_SESSION/hover_run_manifest.txt
hover_failed=0
for repeat in $(seq 1 "$HOVER_REPEATS"); do
  for fixture in \
    arm_tracking_direction_test_paths.json \
    arm_tracking_reversal_test_paths.json \
    arm_tracking_curve_test_paths.json; do
    if ! ros2 launch robross_painter paint.launch.py \
        aubo_type:=$AUBO_TYPE \
        calibration_file:=$HOME/hardware_hover_a4.yaml \
        canvas_file:=$HOME/canvas_hover_10mm.yaml \
        paths_file:=$ROBROSS_REPO/output/$fixture 2>&1 | \
        tee $PHASE2_SESSION/hover_repeat_${repeat}_${fixture%.json}.log; then
      hover_failed=1
      break 2
    fi
  done
done
test "$hover_failed" -eq 0
```

Stop immediately for queue-full warnings, nonzero `servoj_rc`, a timing fault, visible wrist
oscillation toward the paper, unexpected path geometry, or loss of hover clearance. After all
fixtures finish, stop the bag recorder cleanly with Ctrl-C, then stop MoveIt and the driver.

After the driver exits, verify hover telemetry finalized:

```bash
set -euo pipefail
test -f "$PHASE2_SESSION/hover_servoj_calls.csv"
test ! -e "$PHASE2_SESSION/hover_servoj_calls.csv.partial"
tail -n 1 "$PHASE2_SESSION/hover_servoj_calls.csv" | \
  tee "$PHASE2_SESSION/hover_servoj_footer.txt"
grep -F "status=complete" "$PHASE2_SESSION/hover_servoj_footer.txt"
grep -F "dropped=0" "$PHASE2_SESSION/hover_servoj_footer.txt"
```

### 5.5.6 Analyze the hover bag offline

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
  $PHASE2_SESSION/hover_125hz_008s \
  --canvas-file $PHASE2_SESSION/hover_canvas.yaml \
  --calibration-file $PHASE2_SESSION/hover_hardware.yaml \
  --plane-bias-mm 1.0 \
  --csv $PHASE2_SESSION/hover_tracking.csv \
  --servoj-csv $PHASE2_SESSION/hover_servoj_windows.csv | \
  tee $PHASE2_SESSION/hover_summary.md
```

The summary must show `config: t=0.008` and an effective configured rate of 125 Hz. If the config
line is absent, the bag was started too late; repeat the run instead of interpreting timing.

Validate the full-rate call file and screen report-boundary cadence. The report window is 250
cycles at 125 Hz:

```bash
SESSION="$PHASE2_SESSION" python3 - <<'PY'
import csv
import os
import statistics
from pathlib import Path

session = Path(os.environ["SESSION"])
trials = (
    ("hover", session / "hover_servoj_calls.csv", 0.008, 250),
)
for label, path, nominal, window in trials:
    lines = path.read_text().splitlines()
    footer = lines[-1]
    rows = list(csv.DictReader(line for line in lines if not line.startswith("#")))
    if "status=complete" not in footer or "dropped=0" not in footer:
        raise SystemExit(f"{label}: incomplete footer: {footer}")
    seq = [int(row["seq"]) for row in rows]
    if seq != list(range(len(seq))):
        raise SystemExit(f"{label}: sequence gap or nonzero first sequence")
    wall = [float(row["t_wall_s"]) for row in rows]
    call_ros = [float(row["t_call_ros_s"]) for row in rows]
    cycle_ros = [float(row["t_ros_s"]) for row in rows]
    if any(value <= 0.0 for value in call_ros):
        raise SystemExit(f"{label}: invalid ROS call timestamp")
    max_clock_delta = max(abs(call - cycle) for call, cycle in zip(call_ros, cycle_ros))
    if max_clock_delta > 1.0:
        raise SystemExit(f"{label}: ROS/system clock mismatch {max_clock_delta:.6f} s")
    intervals = [b - a for a, b in zip(wall, wall[1:])]
    boundary = [
        intervals[i - 1]
        for i in range(1, len(rows))
        if int(rows[i]["seq"]) % window == 0
    ]
    if not intervals or not boundary:
        raise SystemExit(f"{label}: insufficient rows/report boundaries")
    late_limit = nominal * 1.25
    print(
        f"{label}: rows={len(rows)} mean_ms={statistics.mean(intervals)*1000:.3f} "
        f"max_ms={max(intervals)*1000:.3f} "
        f"boundary_max_ms={max(boundary)*1000:.3f} "
        f"max_clock_delta_s={max_clock_delta:.6f} "
        f"late_limit_ms={late_limit*1000:.3f}"
    )
    if max(boundary) > late_limit:
        raise SystemExit(f"{label}: report-boundary cadence exceeded late limit")
PY
```

Record these results in the session notes:

| Metric | 125 Hz / 8 ms result |
| --- | ---: |
| Effective update rate and percent configured | |
| Period mean / p95 / p99 / maximum | |
| ServoJ RPC mean / maximum | |
| Queue-full events and retries | |
| Non-OK return codes and exceptions | |
| Full-rate command-to-feedback delay median / p95 | |
| Paint-path signed normal mean / p95 / p99 / maximum by repeat | |
| Raw and time-aligned tangential error | |
| Visible movement-synchronized wrist motion | |

### 5.5.7 Screening and interpretation rules

Apply these rules to the selected 125 Hz pair:

- Actual update rate is at least 95 percent of 125 Hz.
- No queue-full event, unexplained ServoJ return code, exception, or timing fault occurred.
- The full-rate call CSV finalizes with a contiguous sequence, zero drops, valid call timestamps,
  and no report-boundary interval above 10 ms.
- Direction, reversal, and alternating-curve fixtures complete above the paper without a safety
  abort or loss of physical hover clearance.
- The executor's Cartesian validation lines satisfy the limit recorded in the exact hover profile.
- Report each deliberate repetition independently. Stable p95/p99 distributions matter more than
  one pooled global maximum, but every outlier still requires inspection.
- Evaluate canvas-normal tracking on `paint_path` separately from `move_to`, `lower_tool`, and
  `lift_tool`. Approach and retreat intentionally move along the canvas normal.
- Treat command-to-feedback latency as a diagnostic. Report raw simultaneous error and
  time-aligned geometric error; the old `<30/<50 ms` threshold is not an automatic contact gate.
- No visible movement-synchronized wrist motion toward the paper, unexpected geometry, or
  unstable spring behavior is acceptable.

Preserve and inspect the executor evidence with:

```bash
grep -h "Cartesian FK error after retiming" $PHASE2_SESSION/hover_repeat_*.log
```

Treat a missing ServoJ configuration, report window, complete full-rate CSV, exact input manifest,
or report-boundary cadence result as **INCOMPLETE**. Preserve both the analyzer's report-window
CSV and the driver's separate full-rate call trail.

The analyzer's `Historical global tracking screen` pools
travel, approach, painting, and retreat and therefore does not by itself approve or reject the
run. Do not increase plane bias or add position/direction-dependent Z compensation to hide a
tracking result. Use the [current status](aubo-painting-current-status-2026-07-31.md) for the current evidence
interpretation and the separate supervised/unattended operating decisions.

## Step 6 — First contact: the 50 mm line

The July 31 supervised contact artwork completed successfully. For a new setup, source change,
calibration change, or tool change, complete and review Step 5.5 first. Then set
`~/hardware_a4.yaml` to `dry_run: false`, keep the reviewed scaling unchanged, return to the
original `~/canvas_calibration.yaml` (never the hover canvas), clear the arm's entire reach
sphere, and keep one hand on the e-stop. This procedure is supervised; unattended contact remains
unapproved.

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=$AUBO_TYPE \
  calibration_file:=$HOME/hardware_a4.yaml \
  canvas_file:=$HOME/canvas_calibration.yaml \
  paths_file:=$ROBROSS_REPO/output/demo_v1_a4_pen/test_line_paths.json
```

**Gate:** ~50 mm horizontal line at (80,140)→(130,140), uniform darkness (fading toward one side
means the taught plane is tilted → re-teach), pen never bottoms out audibly, paper undamaged.
Compare against `output/demo_v1_a4_pen/test_line_preview.svg`.

## Step 7 — Curves and corners

Keep `dry_run: false` and the reviewed scaling unchanged. Run this only after the 50 mm line
passes:

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

Run the full artwork only after the line and curve gates pass:

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=$AUBO_TYPE \
  calibration_file:=$HOME/hardware_a4.yaml \
  canvas_file:=$HOME/canvas_calibration.yaml \
  paths_file:=$ROBROSS_REPO/output/demo_v1_a4_pen/painting_paths.json
```

Compare the result against `output/demo_v1_a4_pen/path_preview.svg`. Raise `velocity_scaling` only after motion
is trusted.

## Session rules

- **Stack restart ⇒ painting restart.** If the driver or move_group restarts mid-run, the
  planning scene is empty — never "resume" a painting, rerun it.
- Start the arm inside the elbow-up band (freedrive/pendant) or the executor aborts before moving.
- A posture or motion-guard rejection ends that attempt. Do not retry through an unconstrained IK
  goal, automatic home pose, or different elbow family.
- Abort with pen down = straight lift only; if the lift fails, jog the pen clear manually before
  doing anything else.
- Never edit safety params (`cartesian_jump_threshold`, backing/claw settings, guard limits)
  mid-session to get past a failure — a failure means the motion could not be verified safe.
