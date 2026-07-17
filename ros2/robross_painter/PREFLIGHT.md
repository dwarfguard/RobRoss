# Hardware Preflight Checklist

Run through this before every real-arm painting session, top to bottom.
It exists because planning only protects against what is modeled and
taught: most real incidents come from a config that no longer matches the
physical setup.

## 1. Config matches the physical setup

- [ ] `calibration_file` is the hardware profile (`hardware_a4.yaml`
      or a copy), not an RViz demo profile.
- [ ] `tool_offset_xyz` / `tool_offset_rpy` match the mounted claw + pen
      (from CAD or measured on the flange). Re-measure after any pen swap.
- [ ] `claw_collision_size_xyz` generously encloses the real claw
      (a few mm of padding), and the pen tip protrudes beyond the box.
      The executor refuses to start if the box would reach the wall at
      pen contact — treat that error as "measure again", not "shrink the
      box until it starts".
- [ ] `canvas_backing_enabled: true`. With `ground_enabled: false` (the
      `hardware_a4.yaml` default) the auto-sized backing patch is the ONLY
      modeled protection for the surface under the paper — confirm the
      real surface extends past the paper by at least
      `canvas_backing_margin_m`, and nothing else (table edge, clamps,
      easel frame) intrudes into the arm's path, because it is not modeled.
- [ ] If the setup re-enables the ground plane (`ground_enabled: true`),
      `ground_z_m` matches the actual mounting surface.
- [ ] `cartesian_jump_threshold` is nonzero (never 0 — 0 disables the
      guard against arm-configuration flips, which execute as unchecked
      sweeps through the robot/ground/wall).
- [ ] `velocity_scaling`/`acceleration_scaling` at 0.1 for first runs.

## 2. Teach the canvas (after ANY paper/wall change)

The pen is spring-loaded with **3.8 mm of compliance**. Use it as the
error budget: teach each corner with the spring compressed roughly
**1.5–2 mm** (about half travel), the same amount at every corner. The
taught plane then sits ~2 mm "into" the wall, so:

- a plane error toward the wall of up to ~1.8 mm still stays within the
  spring's travel (no hard contact, no arm fault);
- a plane error away from the wall of up to ~1.5 mm still leaves ink on
  the paper (no air-drawing).

Checklist:

- [ ] `joint_trajectory_controller` is inactive and
      `joint_state_broadcaster` remains active. Only then enable pendant
      freedrive and run `teach_canvas.py` with the SAME `tool_offset_xyz` as
      the executor config.
- [ ] Touch top-left / top-right / bottom-left with consistent ~1.5–2 mm
      spring compression; record each corner.
- [ ] `save` reports paper size within a few mm of A4 and no skew
      warning. Any warning: re-teach, don't rationalize.
- [ ] After saving, disable pendant freedrive before reactivating
      `joint_trajectory_controller`. Confirm the controller is active before
      planning or executing motion.

## 3. Dry-run the full artwork (after ANY calibration, spin, or artwork change)

```bash
ros2 launch robross_painter paint.launch.py \
  calibration_file:=<hardware yaml> canvas_file:=<taught yaml> \
  paths_file:=<painting_paths.json>        # with dry_run: true
```

- [ ] Completes all commands. The dry run carries each plan's end state
      into the next plan, so it validates one coherent sequence and exposes
      wrist-limit hot spots (they show up as
      `Cartesian path only X% feasible (obstacle or IK configuration
      flip)`). Repeated failures in one canvas region: try a different
      `tool_spin_deg` or move the canvas, don't lower the jump threshold.

## 4. First contact

- [ ] Clear the arm's whole reach sphere. Pen-up travel may use bounded
      joint-space planning when a straight path fails. It avoids modeled
      obstacles only, not people, tripods, cables, or table clutter. One
      hand on the e-stop.
- [ ] Run the 50 mm test line (`test_line_paths.json`) at
      `velocity_scaling: 0.1`, `dry_run: false`.
- [ ] Line darkness is uniform. Fading toward one side/corner means the
      taught plane is tilted relative to the real paper — re-teach.
- [ ] Pen never bottoms out (listen for it; a bottomed spring loads the
      arm sideways on strokes).

## 5. Rules during the session

- **Stack restart ⇒ painting restart.** Collision objects (ground, wall,
  claw box) live in move_group's planning scene and are reconciled with the
  active profile at executor startup. If move_group or the driver restarts
  mid-run, the scene is empty — never "resume" a painting, rerun it.
- **Elbow posture:** startup and every trajectory must remain in the
  configured elbow-up band. An elbow-down start aborts before motion; use
  freedrive or the pendant to place the arm in the approved posture.
- **No posture retries:** a rejected stroke aborts. The executor does not
  pass through all-zero home, switch elbow family, or retry with an
  unconstrained IK goal.
- **Motion guard:** every trajectory logs guarded-joint goal displacement,
  total travel, and maximum sample step. A limit failure is a rejected plan,
  not a parameter-tuning prompt.
- **Abort behavior:** with the pen down, the executor attempts only a
  straight lift before exiting. It never performs a joint-space retreat
  while the pen is touching the paper. If the lift fails, jog the pen clear
  manually before doing anything else.
- **Never edit safety params mid-session** (`cartesian_jump_threshold`,
  backing/claw settings) to "get past" a failure — a failure is the
  system telling you the motion could not be verified safe.
