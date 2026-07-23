# ArUco 绘制区域检测 → AUBO 机械臂定位

```
+-----------------------+
| [] ID:0       ID:1 [] |
|                       |
|      绘图区域          |
|                       |
| [] ID:3       ID:2 [] |
+-----------------------+
```

**流程**: 摄像头检测 4 个 ArUco → 计算绘图区域中心/尺寸/姿态 → 通过 JSON-RPC 发给 AUBO 机械臂

## 快速开始

```bash
# 1. 安装依赖
pip install -r requirements.txt

# 2. 生成并打印标记
python3 aruco_drawing_area.py --generate-markers

# 3. 检测预览（不连接机械臂）
python3 aruco_drawing_area.py --dry-run

# 4. 实际运行（连接机械臂）
python3 aruco_drawing_area.py --robot-ip 192.168.1.100
```

标定文件（用你自己的工具生成）放在当前目录：
- `camera_calib.json` — 相机内参（格式见下方）
- `handeye_calib.txt` — 手眼变换矩阵 T_base_cam，4×4 齐次矩阵

## 标定文件格式

**camera_calib.json:**
```json
{
  "camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
  "dist_coeffs": [[k1, k2, p1, p2, k3]],
  "img_size": [640, 480]
}
```

**handeye_calib.txt:** 4×4 齐次变换矩阵
```
R00 R01 R02 tx
R10 R11 R12 ty
R20 R21 R22 tz
0   0   0   1
```

## 命令

| 参数 | 说明 |
|------|------|
| `--camera-id ID` | 摄像头 ID（默认 0，用 `--list-cameras` 查看可用 ID） |
| `--robot-ip IP` | 机械臂控制器 IP |
| `--marker-size M` | 标记实际边长/米（默认 0.035） |
| `--aruco-dict NAME` | ArUco 字典（默认 4X4_50） |
| `--camera-calib PATH` | 相机标定文件 |
| `--handeye-calib PATH` | 手眼标定文件 |
| `--list-cameras` | 列出所有可用摄像头设备 |
| `--dry-run` | 仅检测，不连机械臂 |
| `--image PATH` | 从图片检测 |
| `--generate-markers` | 生成标记图片 |
| `--test-comm` | 通信测试 |
| `--start-from ID` | 起始绘制点：0\|1\|2\|3 或 "center"（默认 0） |
| `--robross` | 输出 RobRoss 画布标定 YAML 而非直接控制机械臂 |
| `--robross-output PATH` | RobRoss 画布标定输出路径（默认 canvas_calibration.yaml） |

## 机械臂通信 (JSON-RPC)

- 端口: 8899
- 协议: `rob1.MotionControl.moveLine([x,y,z,rx,ry,rz], a, v, blend, duration)`
- API 参考: https://docs.aubo-robotics.cn/arcs_api/zh/

## 实时操作

启动摄像头实时检测后：
- **Enter** → 发送当前坐标到机械臂
- **q** → 退出
