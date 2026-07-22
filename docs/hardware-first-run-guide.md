# Hardware First-Run Guide: Aubo i5 over Ethernet

Step-by-step commands for the first real-arm painting session, from cabling the robot to the
full A4 artwork. Companion to `docs/hardware-test-checklist.md` (what to verify) and
`ros2/robross_painter/PREFLIGHT.md` (run top-to-bottom before every session) — this guide is the
"exact commands" walkthrough; those two remain the authoritative checklists.

## Readiness status (as of 2026-07-17)

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
  aubo_type:=$AUBO_TYPE robot_ip:=<ROBOT_IP> use_fake_hardware:=false
```

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
behind the paper. The 1.8 mm drawing preload is applied in software by `plane_bias_mm`.
Terminal 3 and 4:

```bash
ros2 run robross_painter teach_canvas.py --ros-args \
  -p tool_offset_xyz:="[0.0, -0.0595, 0.0514]" \
  -p plane_bias_mm:=1.8 \
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
