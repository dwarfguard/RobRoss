# Eye-in-Hand Calibration Guide

## What it does

With the camera mounted on the robot's end effector, convert camera coordinates
into robot coordinates.

```
Camera sees ArUco marker → T_cam_marker (camera frame)
Robot reads end pose    → T_base_ee    (base frame)
                           ↓
              T_base_cam = T_base_ee × T_ee_cam
```

**Eye-in-hand calibration** finds `T_ee_cam` (the camera pose in the
end-effector frame) and saves it as a 4×4 matrix in `eye_in_hand_calib.txt`.

---

## Preparation

### Hardware

| Item | Description |
|------|-------------|
| D405 camera | Mounted on the robot's end effector |
| ArUco marker | Print one, mount on cardboard, **fix it on the table** |
| Robot arm | Powered on, can switch to **freedrive / hand-guide mode** |

> **★ No need to touch the marker with the pen tip!** Keep the marker fixed
> and move the arm into different poses; keep the camera at 10–25 cm so the
> marker is clearly visible.

### Check files

```bash
ls -la camera_calib.json    # must exist (D405 intrinsics)
ls -la markers/             # must contain ArUco marker images
```

### Install dependencies

```bash
pip install opencv-python opencv-contrib-python numpy PyYAML
```

### Test communication

```bash
python3 calibrate_eih.py --robot-ip 192.168.32.101 --dry-run
```

Robot communication uses JSON-RPC over TCP, **port 30004** (not 8899).

---

## Procedure

### Step 1: Run the calibration program

```bash
python3 calibrate_eih.py --robot-ip 192.168.32.101 --camera-id <ID>
```

To only preview the procedure:

```bash
python3 calibrate_eih.py --robot-ip 192.168.32.101 --dry-run
```

### Step 2: Collect poses (repeat 6–8 times)

Each time, do the same thing:

```
1. Keep the marker fixed on the table (must not move during calibration!)

2. Put the arm in freedrive mode and drag it above the marker

3. Adjust the pose so the marker is clearly visible in frame
   Keep 10–25 cm distance; vary angle and distance each time

4. Press [Space] to record
   → the program reads two values automatically:
      Robot: current TCP pose T_base_ee
      Camera: marker pose T_cam_marker
   → screen shows "pair 1" "pair 2" ...
```

**Collection tips (determine calibration quality):**

| Requirement | Reason |
|-------------|--------|
| 6–8 poses | Too few cannot be solved reliably |
| Large angle variation (tilt/pitch/rotation) | AX=XB needs rotation information |
| Vary distance (within 10–25 cm) | Constrains the translation components |
| Avoid all-similar orientations | Degrades toward point matching, rotation unsolvable |

Good spread example:

```
Top-down view → right 45° → front-left 30° → near 12 cm
Far 22 cm    → rotated 90° → high tilt   → low tilt
```

### Step 3: Finish calibration

Press `c`.

The program computes and prints:

```
  ┌─ Result: T_ee_cam
  │  R: [[ 0.99,  0.01,  0.05],
  │      [-0.01,  0.99, -0.02],
  │      [-0.05,  0.02,  0.99]]
  │  t: [0.08, -0.01, 0.05]
  │  Pose consistency: mean position error 1.23 mm, mean angle error 0.42°
  │  Max position error: 2.10 mm
  └─
```

**Quality guide (position consistency = marker pose stability in base frame):**

| Mean position error | Mean angle error | Rating |
|---------------------|------------------|--------|
| < 5 mm | < 1° | Excellent, ready to use |
| 5–15 mm | 1–3° | Acceptable, refine with more poses |
| > 15 mm or > 5° | — | Redo — check pose spread |

Confirm to save `eye_in_hand_calib.txt`.

---

## FAQ

### Q: The arm blocks the marker?

No problem. The program caches the last detected marker pose (yellow `◉` on
screen) and uses it on `[Space]`. The marker must not have moved.

### Q: Too close to see / blurry image?

D405 best range is 7–30 cm. **Keep 10–25 cm** — too close blurs, too far
fails detection.

### Q: Large consistency error after calibration?

- Check whether the marker was bumped during the process
- Is the pose angle variation large enough? (all top-down views degrade)
- Is the distance within the D405 best range?

### Q: Cannot connect to the robot?

- Confirm the controller IP (example: 192.168.32.101)
- Port is **30004**, not 8899
- Ping first, then try `--robot-ip` with `--dry-run`

---

## After Calibration

Verify and generate the canvas calibration with `aruco_eih.py`:

```bash
# Method A: preview (no save)
python3 aruco_eih.py --robot-ip 192.168.32.101 --camera-id <ID> --dry-run

# Method B: generate canvas calibration file (for RobRoss ROS 2)
python3 aruco_eih.py --robot-ip 192.168.32.101 --camera-id <ID> --robross
```

Move the arm above the paper and see the green quadrilateral outline the
drawing area → calibration succeeded.

---

## One-line summary

```
Fix the marker → drag the arm through 6–8 poses pressing Space → press c to finish
```
