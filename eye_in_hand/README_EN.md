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
5. **A live quality report is printed after every capture** — press `[c]`
   once the report shows "rotation axes OK + rotation error < 2°"
   (typically 6–8 poses).
6. Press `[c]` to finish → get `eye_in_hand_calib.txt`.

```bash
# Preview the procedure
python3 eye_in_hand/calibrate_eih.py --robot-ip 192.168.1.100 --dry-run

# Actual calibration
python3 eye_in_hand/calibrate_eih.py \
  --robot-ip 192.168.1.100 \
  --camera-id <ID>
```

> **Note: eye-in-hand must NOT use cached data!** `A` (TCP) and `B` (marker
> pose) must be read at the **same instant**. If the arm blocks the marker,
> `[Space]` is rejected — adjust the pose so the marker is visible again.
> Caching works for the fixed-camera version only, not for eye-in-hand.

> **Capture requirements (important)**
> - Keep the camera **10-25 cm** from the marker: D405's recommended working
>   range is 7-30 cm; closer than 7 cm it goes out of focus and solvePnP
>   depth/position errors grow dramatically;
> - Use **moderate viewing angles** (20°-60° between the camera optical axis
>   and the marker normal): too face-on gives poor rotation diversity, too
>   oblique (>70°) degenerates depth estimation for a planar marker;
> - Hold the arm completely still for ~1 s before pressing `[Space]`;
> - If a capture looks bad, press **`[Backspace]` to delete the last pose**
>   and re-capture it.

### Marker size (common pitfall)

solvePnP uses the **black-square** edge length; the program defaults to
`--marker-size 0.02` (20 mm). If your calibration marker is printed larger
(e.g. full-page print with a 40-50 mm black square), you must pass
`--marker-size <measured edge in metres>` — otherwise every distance is
wrong. The typical symptom is "rotation error tiny (<1°) but position error
more than a centimetre".

The program now auto-estimates the real black-square size. If the live
quality report or the final result shows:

```
⚠ Estimated black square ≈ 47 mm (current parameter 20 mm)
```

measure the black square with a ruler and re-run:

```bash
python3 eye_in_hand/calibrate_eih.py \
  --robot-ip 192.168.32.101 \
  --camera-id 6 \
  --marker-size 0.047
```

Note that the two stages use **different markers**:
- `calibrate_eih.py` `--marker-size` = the **big calibration marker**'s
  black square (e.g. 48 mm → `--marker-size 0.048`);
- `aruco_eih.py` canvas detection uses the four **small corner markers**
  (20 mm) on the paper — keep the default `0.02`
  (`start_painting_eih.sh` defaults to 0.02 as well).

### Camera opening method (important)

The program **prefers the pyrealsense2 SDK** to open the D405:

```
[✓] Using pyrealsense2 SDK to open D405 (1280×720, live intrinsics)
```

The SDK path forces 1280×720 (the D405 native mode) and uses the live
intrinsics of that stream (including factory distortion), exactly matching
the calibration file — the most accurate option.

If you see this line, the SDK is unavailable and the program fell back to
OpenCV UVC:

```
[i] pyrealsense2 unavailable or D405 not connected, falling back to OpenCV UVC
```

In UVC mode the D405 often streams 640×480 (a sensor-crop mode) while the
intrinsics are for 1280×720; the program can only *approximately* rescale
the intrinsics, which noticeably degrades calibration and detection
accuracy — install pyrealsense2 and prefer the SDK path:

```bash
pip install pyrealsense2
```

### Live quality report (after every capture)

Once 3 poses are collected, every `[Space]` re-solves and prints:

```
  ┌─ Live quality (4 poses)
  │  Rotation axes: x=0.54 y=0.40 z=0.06  ✓ OK
  │  Consistency: position 0.0 mm, rotation 0.1°
  └─
```

| Metric | Healthy | Problem |
|---|---|---|
| Rotation axes | Components in all three directions (e.g. 0.4/0.3/0.3) | First component > 0.85 = **degenerate**, rotation unreliable |
| Consistency error | Rotation < 2° | > 5° triggers "collect another pose with a different orientation" |

**Degenerate rotation axes are the most common calibration failure**: no
matter how many poses, if the camera stays nearly face-on to the marker (only
translating, or rotating about one direction), the rotation information about
the other axes is missing and the solved rotation error can reach tens of
degrees. Fix: deliberately tilt, rotate and pitch the camera until the
singular values spread across all three directions.

### Save data and offline analysis

Press `[d]` to save the collected A/B data as `eye_in_hand_calib_data.npz`,
then diagnose offline:

```bash
python3 eye_in_hand/analyze_eih_data.py eye_in_hand_calib_data.npz
```

It reports rotation-axis distribution, pose coverage (distance/angle ranges)
and consistency errors, helping distinguish data degeneracy, solvePnP noise
or TCP reading issues.

### Calibration quality

The program validates "marker pose consistency in the base frame":

| Mean position error | Mean angle error | Rating |
|---|---|---|
| < 5 mm | < 1° | Excellent |
| 5–15 mm | 1–3° | Acceptable |
| > 15 mm or > 5° | — | Redo — check pose spread |

### Troubleshooting large calibration errors

If the rotation-axis distribution looks healthy (x/y/z all far from 1.00)
but the consistency error is still large (position > 15 mm or rotation > 5°),
check in this order:

1. **Use the pyrealsense2 SDK path** (it prints `Using pyrealsense2 SDK to
   open D405 (1280×720)`). The D405's UVC 640×480 mode is a sensor crop and
   does not match the 1280×720 factory intrinsics; rescaling them is only an
   approximation. The SDK path forces 1280×720 and reads the live intrinsics
   of that stream, fixing the resolution problem once and for all.
2. **Hold the arm completely still when pressing Space.** The program
   re-reads the TCP pose at the moment of capture, but residual drag motion
   still pollutes the data; settle for ~1 s before pressing.
3. **The camera mount must be rigid.** If the D405 wiggles on the flange,
   every pose has a different `T_ee_cam`, and the error will not converge as
   you add more poses.
4. **The marker must not move.** Bumping the paper or marker board during
   calibration destroys consistency directly.
5. **Do not use all-zero distortion coefficients.** The old
   `read_realsense_intrinsics.py` wrote the factory distortion as zeros;
   re-run it to regenerate `camera_calib.json` (it now writes the factory
   coefficients), or recalibrate with the chessboard. At close range with a
   wide-angle lens, uncorrected distortion causes millimetre-to-centimetre
   position errors.

## Canvas detection (aruco_eih.py)

After calibration, place the paper on the table (ArUco markers on the 4
corners) and move the arm above the paper:

```bash
# List available cameras
python3 eye_in_hand/aruco_eih.py --list-cameras

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
| `analyze_eih_data.py` | Offline analysis of captured data (npz) → diagnose quality |
| `aruco_eih.py` | Live canvas detection → RobRoss canvas YAML |
| `eih_common.py` | Shared local module: detector, calib loading, canvas math, robot comms |
| `calibrate_camera.py` | Camera intrinsics calibration (self-contained) |
| `read_realsense_intrinsics.py` | Read D405 factory intrinsics |
| `markers/` | Printable ArUco markers |
| `eye_in_hand_calib.txt` | Calibration output (4×4 matrix `T_ee_cam`) |

This folder is **fully self-contained**: the detector, camera-calibration
loading, drawing-area computation, robot communication (JSON-RPC) and
canvas-output logic all live in `eih_common.py`. It has no dependency on
`handeye_calibration/` — you can copy this folder alone and run it.

## Notes

- Camera calibration files live in this folder (`camera_calib.json` etc.)
  and are independent from the fixed-camera version.
- Robot communication uses JSON-RPC over TCP, port **30004** (no pyaubo_sdk).
- Eye-in-hand only solves "camera close to the paper"; during painting the arm
  still needs guidance from the fixed-camera flow or manual positioning — this
  folder currently outputs canvas calibration only, it does not drive the
  painting path directly.
