# Eye-in-Hand Camera — Canvas Detection

Mount the D405 on the robot's end effector so the camera can get **close to the
paper** and detect the ArUco markers. The D405 is a short-range camera (best
7–30 cm), so arm-mounting leverages its near-field precision and solves the
fixed-camera problem of failing to detect 20 mm markers beyond 50 cm.

```
      ┌──────────┐
      │  D405    │  ← mounted on the robot's end effector
      └────┬─────┘
           │  30~40cm
      ┌────┴─────┐
      │  A4 paper│
      └──────────┘
```

## Difference from handeye_calibration

| | Fixed camera (handeye_calibration/) | Arm camera (eye_in_hand/) |
|---|---|---|
| Camera position | Desktop mount, fixed | Robot end effector, moves with arm |
| Calibration target | `T_base_cam` (fixed matrix) | `T_ee_cam` (camera in end-effector frame) |
| Calibration method | Point-pair SVD (Kabsch) | **AX=XB** (Park-Martin) |
| Coordinate transform | Fixed `T_base_cam` | Dynamic `T_base_cam = T_base_ee × T_ee_cam` |
| Detection distance | Unreliable beyond ~50 cm | 20–40 cm, stable |
| Field of view | Large, sees whole paper at once | Small, arm must move above paper |

## Calibration (calibrate_eih.py)

Goal: solve `T_ee_cam` — the camera pose in the robot end-effector frame.

```
A_i = T_base_ee_i        (read from robot)
B_i = T_cam_marker_i     (solvePnP from camera)
A_i X B_i = constant     (marker is fixed in base frame)
→ AX = XB, X = T_ee_cam
```

### Procedure

> **★ No need to touch the marker with the pen tip!** Unlike the fixed-camera
> version, eye-in-hand calibration only needs the marker to stay fixed while
> the arm poses in different configurations so the camera can see it.
> The pen tip can be anywhere — you don't even need a pen mounted.

1. **Fix the ArUco marker on the table — it must not move during calibration!**
2. Mount the camera on the end effector; put the arm in freedrive / hand-guide mode.
3. Drag the arm to different poses (vary angle and distance), keeping the marker
   in view at **10–25 cm** (D405 best 7–30 cm; too close blurs, too far
   reintroduces the 50 cm detection problem).
4. Press `[Space]` at each pose (records TCP pose + marker pose automatically).
5. At least 3 poses, ideally **6–8**.
6. Press `[c]` to finish → get `eye_in_hand_calib.txt`.

```bash
# Preview the procedure
python3 eye_in_hand/calibrate_eih.py --robot-ip 192.168.1.100 --dry-run

# Actual calibration
python3 eye_in_hand/calibrate_eih.py \
  --robot-ip 192.168.1.100 \
  --camera-id <ID>
```

When the arm blocks the marker, the program caches the last detected marker
pose (yellow `◉` on screen) and falls back to it on `[Space]` — same logic as
the fixed-camera version.

### Calibration quality

The program validates "marker pose consistency in the base frame":

| Mean position error | Mean angle error | Rating |
|---|---|---|
| < 5 mm | < 1° | Excellent |
| 5–15 mm | 1–3° | Acceptable |
| > 15 mm or > 5° | — | Redo — check pose spread |

## Canvas detection (aruco_eih.py)

After calibration, place the paper on the table (ArUco markers on the 4
corners) and move the arm above the paper:

```bash
# Preview (no save)
python3 eye_in_hand/aruco_eih.py \
  --robot-ip 192.168.1.100 \
  --camera-id <ID> \
  --dry-run

# Generate RobRoss canvas calibration YAML
python3 eye_in_hand/aruco_eih.py \
  --robot-ip 192.168.1.100 \
  --camera-id <ID> \
  --robross
```

The current TCP pose is read live each frame, so the transform is dynamic:

```
T_base_cam = T_base_ee × T_ee_cam
```

The generated YAML has the exact same format as the fixed-camera version
(`canvas_origin_xyz` + `canvas_quat_xyzw`) and can be fed directly to
`paint.launch.py`.

## Files

| File | Purpose |
|---|---|
| `calibrate_eih.py` | AX=XB calibration → `eye_in_hand_calib.txt` |
| `aruco_eih.py` | Live canvas detection → RobRoss canvas YAML |
| `calibrate_camera.py` | Camera intrinsics calibration (same as fixed-camera version) |
| `read_realsense_intrinsics.py` | Read D405 factory intrinsics |
| `markers/` | Printable ArUco markers |
| `eye_in_hand_calib.txt` | Calibration output (4×4 matrix `T_ee_cam`) |

Code reuses the detector, camera-calibration loading, drawing-area computation
and canvas-output logic from `handeye_calibration/aruco_drawing_area.py`.

## Notes

- Camera calibration files are shared with `handeye_calibration/`
  (`camera_calib.json` etc.), also copied here for convenience.
- Robot communication uses JSON-RPC over TCP, port **30004** (no pyaubo_sdk).
- Eye-in-hand only solves "camera close to the paper"; during painting the arm
  still needs guidance from the fixed-camera flow or manual positioning — this
  folder currently outputs canvas calibration only, it does not drive the
  painting path directly.
