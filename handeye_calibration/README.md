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

**handeye_calib.txt:** 4×4 齐次变换矩阵 T_base_cam

把相机坐标变成机械臂基座坐标：

```
p_base = T_base_cam × p_cam

             ┌                 ┐
             │ R00 R01 R02  tx │
T_base_cam = │ R10 R11 R12  ty │
             │ R20 R21 R22  tz │
             │  0   0   0   1  │
             └                 ┘
```

按 **c** 完成标定后，终端显示结果示例：

```
  ┌─ 结果: T_base_cam
  │  R:                                               ← 旋转矩阵 (3×3)
  │    [0.999847, -0.005508,  0.016582]
  │    [0.005686,  0.999970, -0.005982]
  │    [-0.016550,  0.006069,  0.999845]
  │  t: [0.4500, -0.1200, 0.3800]                    ← 平移向量 (米)
  │  RPY: -0.3°, 0.9°, 0.3°                           ← 欧拉角
  │  平均残差: 1.23 mm                                 ← 越小越好
  │  最大残差: 2.10 mm
  └─
```

| 输出项 | 含义 | 单位 |
|--------|------|------|
| **R** | 旋转矩阵 — 相机坐标系到基座坐标系的旋转 | — |
| **t** | 平移向量 — 相机原点在基座坐标中的位置 | 米 |
| **RPY** | 欧拉角 (Roll/Pitch/Yaw)，R 的可读形式 | 度 |
| **平均残差** | 所有标定点的平均重投影误差 | 毫米 |
| **最大残差** | 误差最大的那个点的偏差 | 毫米 |

**残差判断：**

| 残差 | 评价 |
|------|------|
| < 2 mm | 优秀，可直接用 |
| 2–5 mm | 可用，建议补几个点优化 |
| > 5 mm | 重新做 — 检查笔尖是否对准标记中心 |

标定结果保存为纯文本 `handeye_calib.txt`：
```
R00 R01 R02 tx
R10 R11 R12 ty
R20 R21 R22 tz
0   0   0   1
```

## 坐标变换全链路

手眼标定得到 `T_base_cam` 后，完整坐标变换链路如下：

```
摄像头看见纸面点 P
    │
    ▼
P_cam  (相机坐标系)
    │
    │  T_base_cam  (4×4, 手眼标定结果)
    ▼
P_tcp  ── 笔尖目标位置 (基座坐标系)
    │
    ├── [JSON-RPC 直连管道]
    │   moveLine([P_tcp, roll, pitch, yaw])
    │   └→ AUBO 控制器内部：用 TCP 偏移 [0.0595, 0, 0.0514]
    │       将笔尖目标转为法兰轨迹
    │     └→ 法兰运动 → 笔尖到达 P_tcp ✓
    │
    └── [ROS 2 / MoveIt 管道]
        T_ee = T_tip × tool_offset_inv_
        └→ tool_offset_xyz = [0.0595, 0, 0.0514]  (hardware_a4.yaml)
        └→ MoveIt 规划法兰从当前位置到 T_ee
          └→ 法兰运动 → 笔尖到达 P_tcp ✓
```

**关键要求：** AUBO 示教器上配的 TCP 偏移与 `tool_offset_xyz` **必须一致**。
当前值均为 `[0.0595, 0, 0.0514]` 米（笔尖在法兰坐标系中的位置）。

| 组件 | TCP 偏移来源 | 值 |
|------|-------------|---|
| AUBO 控制器 (JSON-RPC) | 示教器配置的 TCP 参数 | `[0.0595, 0, 0.0514]` |
| ROS 2 / MoveIt | `config/hardware_a4.yaml` → `tool_offset_xyz` | `[0.0595, 0, 0.0514]` |

> 修改任意一边的 TCP 值时，**必须同步修改另一边**，否则两条管道的落笔位置会不一致。

## 命令

| 参数 | 说明 |
|------|------|
| `--camera-id ID` | 摄像头 ID（默认 0，用 `--list-cameras` 查看可用 ID） |
| `--robot-ip IP` | 机械臂控制器 IP |
| `--marker-size M` | 标记实际边长/米（默认 0.02，即 20mm） |
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
- **Enter / Space** → 发送当前坐标到机械臂 / 保存画布标定
- **q** → 退出

## 一键启动 (ArUco → ROS 2 画画)

把纸张随便放在桌上，摄像头检测 4 个 ArUco 标记 → 自动生成画布标定 → ROS 2 开始画画。

### 前提（两个终端保持运行）

```bash
# 终端 1: 机器人驱动
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5

# 终端 2: MoveIt 规划器
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=aubo_i5
```

### 用法

```bash
# 终端 3: 一键启动
cd handeye_calibration/

./start_painting.sh \
  --camera-id 2 \
  --paths-file /path/to/painting_paths.json \
  --calibration-file /path/to/hardware_a4.yaml
```

脚本做了两件事：

```
① ArUco 检测纸张
   python3 aruco_drawing_area.py --camera-id 2 --robross
       ↓
   生成 /tmp/robross_canvas_calibration.yaml
       ↓
② ROS 2 画画
   ros2 launch robross_painter paint.launch.py \
     canvas_file:=/tmp/robross_canvas_calibration.yaml
```

### 参数说明

| 参数 | 说明 |
|------|------|
| `--camera-id` | **必选** 摄像头设备号 |
| `--paths-file` | **必选** 绘画路径文件 (painting_paths.json) |
| `--camera-calib` | 相机内参文件 (默认 camera_calib.json) |
| `--handeye-calib` | 手眼标定矩阵 (默认 handeye_calib.txt) |
| `--marker-size` | ArUco 标记边长/米 (默认 0.02，即 20mm) |
| `--robross-output` | 画布标定 YAML 输出路径 (默认 /tmp/...) |
| `--calibration-file` | ROS 2 硬件参数 YAML (默认仅仿真用) |
| `--ros-workspace` | ROS 2 colcon 工作空间 (也支持 \$COLCON_WS) |

### 完整三步流程

```
终端 1 ─── ros2 launch aubo_ros2_driver aubo_control.launch.py
              ↓ 机器人就绪
终端 2 ─── ros2 launch aubo_moveit_config aubo_moveit.launch.py
              ↓ MoveIt 就绪
放好纸 ─── 任何位置、任何角度
              ↓
终端 3 ─── ./start_painting.sh --camera-id 2 --paths-file ...
              │
              ├─ ArUco 检测 4 个标记 → canvas_calibration.yaml
              │   (纸斜着放也自动校正)
              │
              └─ ros2 launch 画画 → 机械臂开始画
```

## 手动分步运行（不用脚本）

如果不想用一键脚本，也可以分两步手动执行：

### Step 1: ArUco 检测纸张 → 生成画布标定

```bash
cd handeye_calibration/

python3 aruco_drawing_area.py \
  --camera-id 2 \
  --robross \
  --robross-output /tmp/canvas_calibration.yaml
```

按 **Enter** 确认检测到 4 个标记后保存。终端输出类似：

```
[✓] RobRoss 画布标定已保存: /tmp/canvas_calibration.yaml
    canvas_origin_xyz: [0.5985, 0.105, 0.15]
    canvas_quat_xyzw:  [0.0, 0.0, 0.0, 1.0]
    画布尺寸: 210.0 x 297.0 mm
```

### Step 2: 启动 ROS 2 画画

```bash
# 先 source ROS 2 工作空间（如果还没做）
source ~/colcon_ws/install/setup.bash

# 启动画画
ros2 launch robross_painter paint.launch.py \
  aubo_type:=aubo_i5 \
  calibration_file:=/path/to/hardware_a4.yaml \
  paths_file:=/path/to/painting_paths.json \
  canvas_file:=/tmp/canvas_calibration.yaml
```

### 预览检测（不生成文件）

只打开摄像头预览，确认 ArUco 标记能被正确识别：

```bash
python3 aruco_drawing_area.py --camera-id 2 --dry-run
```

按 **q** 退出预览。画面中 4 个标记都显示绿色外框和 "✓ 4/4" 即可。

如果不用 ROS 2、只通过 JSON-RPC 直接控制机械臂：

```bash
python3 aruco_drawing_area.py --camera-id 2 --robot-ip 192.168.1.100
```
