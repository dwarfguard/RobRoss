# Hardware First-Run Guide: Aubo i5 over Ethernet

Step-by-step commands for the first real-arm painting session, from cabling the robot to the
full A4 artwork. Companion to `docs/hardware-test-checklist.md` (what to verify) and
`ros2/robross_painter/PREFLIGHT.md` (run top-to-bottom before every session) — this guide is the
"exact commands" walkthrough; those two remain the authoritative checklists.

## Readiness status (as of 2026-07-15)

**Ready:**
- Workspace built: `install/` contains all packages (`aubo_ros2_driver`, `aubo_moveit_config`,
  `robross_painter`, …).
- Path files generated in `output/`: `painting_paths.json` (46 commands, validated) and
  `test_line_paths.json` (the 50 mm first-contact line) + previews.
- `~/hardware_a4.yaml` exists (copy of `ros2/robross_painter/config/hardware_a4.yaml`);
  `tool_offset_xyz: [0.0595, 0.0, 0.0514]` matches the value used during teaching.
- `velocity_scaling`/`acceleration_scaling` at 0.1 (correct for first runs);
  `cartesian_jump_threshold: 2.0` (nonzero, correct); `canvas_backing_enabled: true`.
- `dry_run: true` in both the shipped profile and `~/hardware_a4.yaml`.

**Gaps to fix before first contact (in order):**
1. Claw collision box mismatch: the hardware profile has
   `claw_collision_size_xyz: [0.06, 0.02, 0.02]` (the template's TODO example) while
   `rviz_taught_a4.yaml` has `[0.118, 0.075, 0.049]` with offset `[0.0184, 0.0, 0.0245]`, which
   looks like real measured values → confirm against the real claw and copy the measured box into
   `~/hardware_a4.yaml`. PREFLIGHT: the box must generously enclose the claw, pen tip protruding
   beyond it.
2. `~/canvas_calibration.yaml` is invalid (measured 366.7 × 315.8 mm, corner skew 23.75°; an A4
   is 210 × 297 mm, skew must be < 2°) → re-teach on the real paper (Step 4). This bad teach —
   not the elbow constraints — is what caused the RViz `wrist3_joint` motion-guard abort.
3. Robot IP unknown — read it off the teach pendant (Settings → Network) once cabled.

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
  aubo_type:=$AUBO_TYPE robot_ip:=<ROBOT_IP> use_fake_hardware:=false
```

Terminal 2 (MoveIt):
```bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=$AUBO_TYPE
```

Sanity check (terminal 3): `ros2 topic echo /joint_states --once` shows live joint angles that
change when the arm is jogged. (`aubo_client.launch.py` is a separate service demo — not needed
for this flow.)

## Step 4 — Teach the canvas (real paper, freedrive)

The pen spring has 3.8 mm of compliance — touch each corner with ~1.5–2 mm compression,
consistent at every corner. Put the pendant into freedrive, then (terminal 3):

```bash
ros2 run robross_painter teach_canvas.py --ros-args \
  -p tool_offset_xyz:="[0.0595, 0.0, 0.0514]" \
  -p output_file:=$HOME/canvas_calibration.yaml
```

Per corner: freedrive to ~10 mm out (freedrive breakaway force is too high for accurate small
motions), disable freedrive, finish the approach with the pendant's slowest jog, hands off the
arm, then record. A record is rejected if the arm moved in the last second — release, let it
settle, re-record. Bottom-right is a validation-only corner; `save` warns if it sits > 2 mm
from where the other three predict it:

```bash
ros2 service call /teach_canvas/record_top_left     std_srvs/srv/Trigger
ros2 service call /teach_canvas/record_top_right    std_srvs/srv/Trigger
ros2 service call /teach_canvas/record_bottom_left  std_srvs/srv/Trigger
ros2 service call /teach_canvas/record_bottom_right std_srvs/srv/Trigger
ros2 service call /teach_canvas/save                std_srvs/srv/Trigger
```

**Gate:** `save` must report ≈210 × 297 mm, no skew warning (< 2°), and no bottom-right
residual warning. Any warning → re-teach; don't rationalize.

## Step 5 — Dry-run everything (`dry_run: true` in `~/hardware_a4.yaml`)

Full artwork, then the test line:

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=$AUBO_TYPE \
  calibration_file:=$HOME/hardware_a4.yaml \
  canvas_file:=$HOME/canvas_calibration.yaml \
  paths_file:=$ROBROSS_REPO/output/painting_paths.json
# then again with paths_file:=$ROBROSS_REPO/output/test_line_paths.json
```

**Gate:** all commands plan cleanly, arm never moves. Repeated
`Cartesian path only X% feasible` in one canvas region → try a different `tool_spin_deg` or move
the canvas; never lower `cartesian_jump_threshold`. A motion-guard rejection is a rejected plan,
not a parameter-tuning prompt.

## Step 6 — First contact: the 50 mm line

Edit `~/hardware_a4.yaml`: `dry_run: false` (keep scaling at 0.1). Clear the arm's entire reach
sphere; one hand on the e-stop:

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

## Step 7 — Full artwork

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
