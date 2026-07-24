# ArUco Drawing Area Detection → AUBO Robot Arm Localization

```
+-----------------------+
| [] ID:0       ID:1 [] |
|                       |
|      Drawing Area     |
|                       |
| [] ID:3       ID:2 [] |
+-----------------------+
```

**Pipeline**: Camera detects 4 ArUco markers → compute drawing area center/size/orientation → send to AUBO arm via JSON-RPC

## Quick Start

```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Generate and print markers
python3 aruco_drawing_area.py --generate-markers

# 3. Preview detection (no robot connection)
python3 aruco_drawing_area.py --dry-run

# 4. Run live (connect to robot)
python3 aruco_drawing_area.py --robot-ip 192.168.1.100
```

Calibration files (place in this directory):
- `camera_calib.json` — Camera intrinsics (format below)
- `handeye_calib.txt` — Hand-eye transform matrix T_base_cam, 4×4 homogeneous matrix

## Calibration File Formats

**camera_calib.json:**
```json
{
  "camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
  "dist_coeffs": [[k1, k2, p1, p2, k3]],
  "img_size": [640, 480]
}
```

**handeye_calib.txt:** 4×4 homogeneous transform matrix T_base_cam

Transforms a point from camera coordinates to robot base coordinates:

```
p_base = T_base_cam × p_cam

        ┌                 ┐
        │ R00 R01 R02  tx │
T_base_cam = │ R10 R11 R12  ty │
        │ R20 R21 R22  tz │
        │  0   0   0   1 │
        └                 ┘
```

After pressing **c** to complete calibration, the terminal shows:

```
  ┌─ Result: T_base_cam
  │  R:                                               ← rotation matrix (3×3)
  │    [0.999847, -0.005508,  0.016582]
  │    [0.005686,  0.999970, -0.005982]
  │    [-0.016550,  0.006069,  0.999845]
  │  t: [0.4500, -0.1200, 0.3800]                    ← translation vector (meters)
  │  RPY: -0.3°, 0.9°, 0.3°                           ← Euler angles
  │  Mean residual: 1.23 mm                           ← smaller is better
  │  Max residual:  2.10 mm
  └─
```

| Output | Meaning | Unit |
|--------|---------|------|
| **R** | Rotation matrix — camera → base orientation | — |
| **t** | Translation — camera origin in base coordinates | meters |
| **RPY** | Euler angles (Roll/Pitch/Yaw), human-readable R | degrees |
| **Mean residual** | Average reprojection error across all points | mm |
| **Max residual** | Worst point's deviation | mm |

**Residual quality guide:**

| Residual | Rating |
|----------|--------|
| < 2 mm | Excellent, ready to use |
| 2–5 mm | Acceptable, can refine with more points |
| > 5 mm | Redo — check if the pen tip was aligned to the marker center |

Saved as plain text `handeye_calib.txt`:
```
R00 R01 R02 tx
R10 R11 R12 ty
R20 R21 R22 tz
0   0   0   1
```

## Full Coordinate Chain

After hand-eye calibration produces `T_base_cam`, the complete coordinate pipeline is:

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
    │   └→ AUBO controller internally: uses TCP offset [0.0595, 0, 0.0514]
    │       to convert pen-tip target to flange trajectory
    │     └→ Flange moves → pen tip lands on P_tcp ✓
    │
    └── [ROS 2 / MoveIt Pipeline]
        T_ee = T_tip × tool_offset_inv_
        └→ tool_offset_xyz = [0.0595, 0, 0.0514]  (hardware_a4.yaml)
        └→ MoveIt plans flange motion from current pose to T_ee
          └→ Flange moves → pen tip lands on P_tcp ✓
```

**Critical requirement:** The TCP offset set on the AUBO teach pendant MUST match `tool_offset_xyz`.
Current value for both: `[0.0595, 0, 0.0514]` meters (pen tip in flange coordinates).

| Component | TCP offset source | Value |
|-----------|------------------|-------|
| AUBO controller (JSON-RPC) | Teach pendant TCP config | `[0.0595, 0, 0.0514]` |
| ROS 2 / MoveIt | `config/hardware_a4.yaml` → `tool_offset_xyz` | `[0.0595, 0, 0.0514]` |

> If you modify the TCP value on either side, **the other side must be updated too**, or the two pipelines will place the pen at different positions.

## Options

| Argument | Description |
|----------|-------------|
| `--camera-id ID` | Camera device ID (default 0, use `--list-cameras` to find available IDs) |
| `--robot-ip IP` | Robot controller IP address |
| `--marker-size M` | Actual marker side length in meters (default 0.035) |
| `--aruco-dict NAME` | ArUco dictionary (default 4X4_50) |
| `--camera-calib PATH` | Camera calibration file |
| `--handeye-calib PATH` | Hand-eye calibration file |
| `--list-cameras` | List all available camera devices |
| `--dry-run` | Detection only, no robot connection |
| `--image PATH` | Process a single image file |
| `--generate-markers` | Generate marker images for printing |
| `--test-comm` | Test robot communication only |
| `--start-from ID` | Starting corner: 0\|1\|2\|3 or "center" (default 0) |
| `--robross` | Output RobRoss canvas calibration YAML instead of direct robot control |
| `--robross-output PATH` | Output path for RobRoss canvas calibration (default canvas_calibration.yaml) |
| `--start-from ID` | Starting corner: 0\|1\|2\|3 or "center" (default 0) |
| `--robross` | Output RobRoss canvas calibration YAML instead of direct robot control |
| `--robross-output PATH` | Output path for RobRoss canvas calibration (default canvas_calibration.yaml) |

## Robot Communication (JSON-RPC)

- Port: 8899
- Protocol: `rob1.MotionControl.moveLine([x,y,z,rx,ry,rz], a, v, blend, duration)`
- API Reference: https://docs.aubo-robotics.cn/arcs_api/en/

## Live Operation

After starting the camera live-feed:
- **Enter / Space** — Save canvas calibration / Send coordinates to robot
- **q** — Quit

## One-Click Startup (ArUco → ROS 2 Painting)

Place the paper anywhere on the table. The camera detects 4 ArUco markers, generates canvas calibration, and ROS 2 starts painting — all in one command.

### Prerequisites (keep both terminals running)

```bash
# Terminal 1: Robot driver
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5

# Terminal 2: MoveIt planner
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=aubo_i5
```

### Usage

```bash
# Terminal 3: One-click start
cd handeye_calibration/

./start_painting.sh \
  --camera-id 2 \
  --paths-file /path/to/painting_paths.json \
  --calibration-file /path/to/hardware_a4.yaml
```

The script does two things:

```
① ArUco paper detection
   python3 aruco_drawing_area.py --camera-id 2 --robross
       ↓
   Generates /tmp/robross_canvas_calibration.yaml
       ↓
② ROS 2 painting
   ros2 launch robross_painter paint.launch.py \
     canvas_file:=/tmp/robross_canvas_calibration.yaml
```

### Arguments

| Argument | Description |
|----------|-------------|
| `--camera-id` | **Required** Camera device ID |
| `--paths-file` | **Required** Path to painting_paths.json |
| `--camera-calib` | Camera intrinsics (default camera_calib.json) |
| `--handeye-calib` | Hand-eye calibration (default handeye_calib.txt) |
| `--marker-size` | ArUco marker side length in meters (default 0.035) |
| `--robross-output` | Canvas YAML output path (default /tmp/...) |
| `--calibration-file` | ROS 2 hardware parameters YAML |
| `--ros-workspace` | ROS 2 colcon workspace (also reads \$COLCON_WS) |

### Full Workflow

```
Terminal 1 ─── ros2 launch aubo_ros2_driver aubo_control.launch.py
                   ↓ robot ready
Terminal 2 ─── ros2 launch aubo_moveit_config aubo_moveit.launch.py
                   ↓ MoveIt ready
Place paper ─── Any position, any angle
                   ↓
Terminal 3 ─── ./start_painting.sh --camera-id 2 --paths-file ...
                   │
                   ├─ ArUco detects 4 markers → canvas_calibration.yaml
                   │   (automatically corrects for paper rotation)
                   │
                   └─ ros2 launch → robot starts painting
```

## Manual Step-by-Step (without the script)

If you prefer to run each step separately instead of using `start_painting.sh`:

### Step 1: ArUco detection → canvas calibration YAML

```bash
cd handeye_calibration/

python3 aruco_drawing_area.py \
  --camera-id 2 \
  --robross \
  --robross-output /tmp/canvas_calibration.yaml
```

Press **Enter** to confirm the detection and save. Output looks like:

```
[✓] RobRoss canvas calibration saved: /tmp/canvas_calibration.yaml
    canvas_origin_xyz: [0.5985, 0.105, 0.15]
    canvas_quat_xyzw:  [0.0, 0.0, 0.0, 1.0]
    Canvas size: 210.0 x 297.0 mm
```

### Step 2: Start ROS 2 painting

```bash
# Source ROS 2 workspace first (if not already done)
source ~/colcon_ws/install/setup.bash

# Start painting
ros2 launch robross_painter paint.launch.py \
  aubo_type:=aubo_i5 \
  calibration_file:=/path/to/hardware_a4.yaml \
  paths_file:=/path/to/painting_paths.json \
  canvas_file:=/tmp/canvas_calibration.yaml
```

### Preview detection (no file output)

Open the camera preview to verify ArUco markers are detected correctly:

```bash
python3 aruco_drawing_area.py --camera-id 2 --dry-run
```

Press **q** to exit. All 4 markers should show green outlines and "✓ 4/4" in the corner.

For direct robot control via JSON-RPC (without ROS 2):

```bash
python3 aruco_drawing_area.py --camera-id 2 --robot-ip 192.168.1.100
```
