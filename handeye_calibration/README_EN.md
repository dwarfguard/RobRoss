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

**handeye_calib.txt:** 4×4 homogeneous transform matrix
```
R00 R01 R02 tx
R10 R11 R12 ty
R20 R21 R22 tz
0   0   0   1
```

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
- **Enter / Space** → Send current coordinates to the robot
- **q** → Quit

## Integration with RobRoss ROS 2

```bash
# Generate canvas calibration YAML (for ROS 2 painting_executor)
python3 aruco_drawing_area.py --camera-id 2 --robross --robross-output canvas_calibration.yaml

# Then use with RobRoss paint launch:
ros2 launch robross_painter paint.launch.py \
  aubo_type:=aubo_i5 \
  paths_file:=<your_paths.json> \
  canvas_file:=canvas_calibration.yaml
```
