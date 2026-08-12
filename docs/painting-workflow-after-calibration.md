# Eye-in-Hand Localization After Calibration

**Status:** Conceptual overview, not a real-arm run procedure

**Canonical calibration guide:** `../eye_in_hand/README_EN.md`

**Canonical execution procedure:** `hardware_run_guide.md`

Eye-in-hand calibration records the camera transform relative to the robot
controller's active TCP. Despite the historical `T_ee_cam` name, changing the
camera mount or the controller TCP definition invalidates the calibration.

## Session Flow

1. The operator positions the arm so all four canvas markers are visible.
   `aruco_eih.py` reads robot state but does not move the arm.
2. `aruco_eih.py --robross` localizes the marker-defined canvas and writes its
   origin and orientation to YAML.
3. The generated YAML is reviewed as localization evidence. It is not contact
   authorization and must not bypass the hardware guide's dimension, flatness,
   dry-run, hover, test-line, or curve gates.
4. An independently generated `painting_paths.json` supplies millimeter canvas
   coordinates. The painter combines those coordinates with the qualified
   canvas frame and tool transform.

```text
painting_paths.json (canvas millimeters)
    + qualified canvas pose
    + reviewed tool and collision geometry
    -> pen-tip poses
    -> end-effector poses
    -> MoveIt trajectories
```

## Localize The Canvas

Run from the repository root after completing the calibration procedure:

```bash
python3 eye_in_hand/aruco_eih.py \
  --robot-ip <controller-ip> \
  --camera-id <camera-id> \
  --robross
```

The operator, not this program, positions the arm. Confirm that the reported
marker-defined width and height match the path canvas within the tolerance used
by `docs/hardware_run_guide.md`. Current generated YAML records dimensions only
in comments, so this check is manual and fail-closed.

Do not launch contact motion directly from camera localization. Continue with
`docs/hardware_run_guide.md`, using its calibrated robot model, reviewed
hardware-profile copy, canvas qualification, and staged execution gates.

## Recalibration Triggers

- Camera mount position or orientation changes.
- The controller's active TCP definition changes.
- Camera intrinsics or stream mode changes.
- Calibration quality no longer meets `eye_in_hand/README_EN.md`.

Moving or rotating the paper requires canvas localization again, but does not
by itself require camera recalibration.
