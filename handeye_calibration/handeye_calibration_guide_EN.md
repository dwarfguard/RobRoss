# Hand-Eye Calibration Guide

## What It's For

Converts coordinates seen by the camera into coordinates the robot arm understands.

```
Camera sees ArUco marker → [0.12, -0.05, 0.35] (camera coordinates)
                         → ?                     (robot base coordinates)
```

**Hand-eye calibration** solves for this "?", producing a 4×4 matrix `T_base_cam` saved to `handeye_calib.txt`.

---

## Prerequisites

### Hardware

| Item | Description |
|------|-------------|
| D405 camera | Mounted securely, must not move. Aimed at the drawing area |
| ArUco marker | Generate with `--generate-markers`, print one, mount on cardboard |
| Robot arm | Powered on, able to switch to **freedrive / hand-guide mode** |
| Pen tip | Installed at the arm's end, TCP calibrated |

### Verify Files

```bash
ls -la camera_calib.json    # Must exist (stores D405 intrinsics)
ls -la markers/             # Must contain ArUco marker images
```

### Install Dependencies

```bash
pip install opencv-python opencv-contrib-python numpy
```

### Test Communication

```bash
python3 aruco_drawing_area.py --test-comm --robot-ip 192.168.1.100
# Should see current TCP pose → communication OK
```

---

## Procedure

### Step 1: Start the Calibration Program

```bash
python3 calibrate_handeye.py --robot-ip 192.168.1.100
```

To preview the workflow without connecting:

```bash
python3 calibrate_handeye.py --robot-ip 192.168.1.100 --dry-run
```

### Step 2: Collect Point Pairs (Repeat 5-6 Times)

Do the same thing each time:

```
1. Place the ArUco marker somewhere on the table

2. Switch the robot to freedrive mode

3. Push the arm so the pen tip precisely touches the marker center ⬇️
         ┌──────┐
         │  ◉   │  ← touch here with pen tip
         └──────┘
        ArUco marker

4. Press [Space] on the keyboard to record
   → The program automatically reads two values:
      Camera:    marker center position in camera coordinates
      Robot arm: TCP position in base coordinates
   → Screen shows "Pair 1" "Pair 2" ...
```

**Tips for good calibration quality:**

| Requirement | Reason |
|-------------|--------|
| 5-6 points | Too few points gives inaccurate results |
| Spread across height and depth | Points on one plane miss Z-direction information |
| Don't put them on a line | Collinear points can't solve for rotation |
| Cover the full camera field of view | Ensures accuracy across the whole workspace |

Good distribution example:

```
  z ↑          · (high, far)
    |    · (mid, left)     · (mid, right)
    |       · (low, near)
    +——————————————→ x/y
```

### Step 3: Complete Calibration

Press `c`.

The program computes and displays:

```
  ┌─ Result: T_base_cam
  │  R: [[ 0.99,  0.01,  0.05],     ← rotation matrix (3×3)
  │      [-0.01,  0.99, -0.02],
  │      [-0.05,  0.02,  0.99]]
  │  t: [0.45, -0.12, 0.38]         ← translation vector (meters)
  │  Mean residual: 1.23 mm         ← smaller is better
  │  Max residual:  2.10 mm
  └─
```

**What is T_base_cam?**

A 4×4 homogeneous transform matrix that converts camera coordinates to robot base coordinates:

```
p_base = T_base_cam × p_cam
  ↑                        ↑
base coords              camera coords
```

```
        ┌                 ┐
        │ R00 R01 R02  tx │
T_base_cam = │ R10 R11 R12  ty │
        │ R20 R21 R22  tz │
        │  0   0   0   1 │
        └                 ┘
```

| Part | Meaning |
|------|---------|
| **R** | Rotation — camera orientation expressed in the base frame |
| **t** | Translation — camera origin position in the base frame |
| **RPY** | Euler angles, human-readable form of R |

Saved as `handeye_calib.txt`, the raw 4×4 matrix:

```
R00 R01 R02 tx
R10 R11 R12 ty
R20 R21 R22 tz
0   0   0   1
```

**Calibration quality guide:**

| Residual | Rating |
|----------|--------|
| < 2 mm | Excellent, ready to use |
| 2-5 mm | Acceptable, can refine with more points |
| > 5 mm | Redo — check if the pen tip was aligned to the marker center |

Press `y` to save. You get `handeye_calib.txt`.

---

## Full Coordinate Chain

After hand-eye calibration produces `T_base_cam`, the complete coordinate pipeline for drawing is:

```
Camera sees paper point P
    │
    ▼
P_cam  (camera coordinates)
    │
    │  T_base_cam  (4×4, hand-eye calibration result)
    ▼
P_tcp  ── pen tip target position (base coordinates)
    │
    ├── [JSON-RPC Direct Pipeline]
    │   moveLine([P_tcp, roll, pitch, yaw])
    │   └→ AUBO controller uses TCP offset to convert pen tip target to flange trajectory
    │
    └── [ROS 2 / MoveIt Pipeline]
        T_ee = T_tip × tool_offset_inv_
        └→ tool_offset_xyz describes the pen tip in flange coordinates
        └→ MoveIt plans flange motion
```

**T_base_cam maps to the pen tip, not the flange.** During calibration, `get_tcp_pose()` reads the pen tip (TCP) position from the robot. Then either the AUBO controller (JSON-RPC pipeline) or `tool_offset_inv_` (ROS 2 pipeline) converts the pen tip target to the flange center trajectory.

> The TCP offset set on the AUBO teach pendant MUST match `tool_offset_xyz` in `config/hardware_a4.yaml`.
> Current value: `[0.0595, 0, 0.0514]` meters.

---

## FAQ

### Q: The pen tip can't reach the marker center?

- Use a larger marker (adjust the `--marker-size` argument)
- Mount the marker on stiff cardboard instead of soft paper
- If the tip is too thick, use one corner of the ArUco marker instead of its center

### Q: Large residual after calibration?

- Check that each point was truly aligned (offline review: look at saved images)
- Check whether the points are too clustered together
- Verify that the robot's TCP calibration is accurate

### Q: Can I calibrate without a robot arm?

Yes, use `--manual` mode to enter base coordinates by hand:

```bash
python3 calibrate_handeye.py --manual
```

But you'll need to measure the marker positions relative to the robot base manually — limited accuracy.

### Q: What if the camera is moved?

**Recalibrate.** Once the camera position changes, `handeye_calib.txt` is invalid.

---

## After Calibration

Verify with `aruco_drawing_area.py`:

```bash
# Method A: Direct robot control
python3 aruco_drawing_area.py --camera-id 2 --robot-ip 192.168.1.100 --start-from 0

# Method B: Generate canvas calibration file (for RobRoss ROS 2)
python3 aruco_drawing_area.py --camera-id 2 --robross --robross-output canvas_calibration.yaml
```

If the robot arm accurately moves to the drawing area corners → calibration successful.

---

## One-Line Summary

```
Run calibrate_handeye.py → touch marker 5 times, press Space → press c to finish
```
