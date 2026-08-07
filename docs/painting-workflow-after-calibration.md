# 标定完成后的画画流程 / Painting Workflow After Calibration

**Status:** Workflow reference
**Last updated:** 2026-08-06
**Applies to:** eye-in-hand camera path (`eye_in_hand/`)

---

# 中文版

## 标定完成了什么

手眼标定（`calibrate_eih.py`）产出一个文件：`eye_in_hand/eye_in_hand_calib.txt`，
里面是 4×4 矩阵 `T_ee_cam` —— **相机在机械臂末端坐标系中的固定位姿**。
它描述的是“相机装在臂上哪个位置、朝哪个方向”，所以只在**相机装法改变时**
才需要重做。

标定完成后，每次画画分三步：

## 第 1 步：画布检测（每次放纸都要做）

```bash
cd eye_in_hand/
python3 aruco_eih.py \
  --robot-ip 192.168.32.101 \
  --camera-id <ID> \
  --robross
```

做了什么：

1. 机械臂移动到纸面上方（相机离纸 20~40cm），4 个角上的小 ArUco 标记进入画面；
2. 程序**实时读取机械臂 TCP**（`T_base_ee`），配合标定好的 `T_ee_cam` 动态算出：

```
T_base_cam = T_base_ee × T_ee_cam
```

3. 识别 4 个标记后，在基座坐标系里算出纸面的位置和朝向：
   - 原点 = ID:0 内侧顶点（纸的左上角）
   - X 轴 = ID:0 → ID:1（纸面右方向）
   - Y 轴 = ID:0 → ID:3（纸面向下）
   - Z 轴 = 垂直于纸面
4. 按 `[Enter]` 保存成 YAML（默认 `canvas_calibration_eih.yaml`），里面是
   `canvas_origin_xyz` + `canvas_quat_xyzw` —— **这张纸现在在机器人坐标系里“定位”了**。

> 关键点：这步每次纸的位置/角度变了都要重做，因为它输出的是“纸现在在哪”。
> 而 `T_ee_cam` 不用重做。

## 第 2 步：路径文件（画画内容，离线生成）

`painting_paths.json` 是提前生成的绘画指令，坐标以**纸的左上角为原点**、单位毫米：

```json
{ "command": "move_to", "x_mm": 58.08, "y_mm": 10.0 }
```

它和纸放哪里无关 —— 纸斜着放、挪个位置，路径数字不用改。

## 第 3 步：ROS 2 执行（把两者合起来）

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=aubo_i5 \
  paths_file:=<painting_paths.json> \
  canvas_file:=<canvas_calibration_eih.yaml> \
  calibration_file:=<hardware_a4.yaml>
```

> ⚠ `paint.launch.py` 默认的 `calibration_file` 是 `rviz_wall_a4.yaml`（仅仿真）。
> 真机画画**必须**显式传 `hardware_a4.yaml`（含 `tool_offset_xyz: [0.001208, -0.06034, 0.090753]`）。

`painting_executor` 做的事：

```
painting_paths.json（画布毫米坐标）
  → CanvasFrame: 毫米 → 基座坐标（用第 1 步的 YAML）
  → 笔尖位姿 → 末端位姿（用 tool_offset_xyz 换算）
  → MoveIt 轨迹 → joint_trajectory_controller
  → aubo servoJoint() → 机械臂动笔
```

也就是：**路径文件告诉它“往右 58mm、往下 10mm”，画布 YAML 告诉它“纸的左上角
在哪、纸面朝向哪”，两者一乘，机械臂就知道该把笔放到物理世界的哪个点。**

## 一键启动（合并第 1 步和第 3 步）

```bash
cd eye_in_hand/
./start_painting_eih.sh \
  --robot-ip 192.168.32.101 \
  --camera-id <ID> \
  --paths-file <painting_paths.json> \
  --calibration-file <hardware_a4.yaml>
```

脚本先运行 `aruco_eih.py --robross` 生成画布 YAML（默认 `/tmp/robross_canvas_calibration_eih.yaml`），
再自动 `ros2 launch` 开始画画。

## 完整时序

| 时机 | 做什么 | 产物 | 多久做一次 |
|---|---|---|---|
| 相机装法改变时 | 手眼标定 `calibrate_eih.py` | `eye_in_hand_calib.txt`（T_ee_cam） | 一次性 |
| 每次放纸 | 画布检测 `aruco_eih.py --robross` | `canvas_calibration_eih.yaml` | 每次画画前 |
| 离线 | 生成路径 | `painting_paths.json` | 内容变化时 |
| 执行 | `ros2 launch ... paint.launch.py` | 机械臂作画 | 每次画画 |

## 一句话总结

**标定解决“相机和臂的关系”，画布检测解决“纸和机器人的关系”，路径文件解决
“画什么”** —— 三者结合，纸放在哪、斜不斜都能画。

---

# English Version

## What Calibration Produces

Hand-eye calibration (`calibrate_eih.py`) produces one file:
`eye_in_hand/eye_in_hand_calib.txt`, a 4×4 matrix `T_ee_cam` — the **fixed pose of
the camera in the robot end-effector frame**. It describes "where the camera is
mounted on the arm and which way it points", so it only needs to be redone when
**the camera mounting changes**.

After calibration, each painting session has three steps:

## Step 1: Canvas Detection (every time the paper is placed)

```bash
cd eye_in_hand/
python3 aruco_eih.py \
  --robot-ip 192.168.32.101 \
  --camera-id <ID> \
  --robross
```

What happens:

1. The arm moves above the paper (camera 20–40 cm away) so the four small ArUco
   corner markers are in view.
2. The program **reads the robot TCP in real time** (`T_base_ee`) and combines it
   with the calibrated `T_ee_cam`:

```
T_base_cam = T_base_ee × T_ee_cam
```

3. After detecting the 4 markers, it computes the paper plane in base coordinates:
   - Origin = inner corner of ID:0 (paper top-left)
   - X axis = ID:0 → ID:1 (paper right)
   - Y axis = ID:0 → ID:3 (paper down)
   - Z axis = perpendicular to the paper
4. Press `[Enter]` to save the YAML (default `canvas_calibration_eih.yaml`)
   containing `canvas_origin_xyz` + `canvas_quat_xyzw` — **the paper is now
   localized in the robot coordinate system**.

> Key point: this step must be redone whenever the paper moves or rotates,
> because its output is "where the paper is right now". `T_ee_cam` does not
> need to be redone.

## Step 2: Path File (what to draw, generated offline)

`painting_paths.json` is the pre-generated painting command file. Coordinates
use **millimeters with the paper top-left as origin**:

```json
{ "command": "move_to", "x_mm": 58.08, "y_mm": 10.0 }
```

It is independent of where the paper is — the numbers do not change whether the
paper is moved or rotated.

## Step 3: ROS 2 Execution (combines the two)

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=aubo_i5 \
  paths_file:=<painting_paths.json> \
  canvas_file:=<canvas_calibration_eih.yaml> \
  calibration_file:=<hardware_a4.yaml>
```

> ⚠ The default `calibration_file` of `paint.launch.py` is `rviz_wall_a4.yaml`
> (simulation only). For real hardware you **must** pass `hardware_a4.yaml`
> explicitly (it contains `tool_offset_xyz: [0.001208, -0.06034, 0.090753]`).

What `painting_executor` does:

```
painting_paths.json (canvas mm coordinates)
  → CanvasFrame: mm → base coordinates (using the Step 1 YAML)
  → pen-tip pose → end-effector pose (via tool_offset_xyz)
  → MoveIt trajectory → joint_trajectory_controller
  → aubo servoJoint() → the arm draws
```

In other words: the **path file says "58 mm right, 10 mm down"**, and the
**canvas YAML says "where the paper's top-left corner is and which way the
paper faces"**. Multiply them and the arm knows exactly where to put the pen in
the physical world.

## One-Click Startup (combines Steps 1 and 3)

```bash
cd eye_in_hand/
./start_painting_eih.sh \
  --robot-ip 192.168.32.101 \
  --camera-id <ID> \
  --paths-file <painting_paths.json> \
  --calibration-file <hardware_a4.yaml>
```

The script runs `aruco_eih.py --robross` to generate the canvas YAML (default
`/tmp/robross_canvas_calibration_eih.yaml`), then automatically launches ROS 2
to paint.

## Full Timeline

| When | What | Output | How often |
|---|---|---|---|
| Camera mounting changes | Hand-eye calibration `calibrate_eih.py` | `eye_in_hand_calib.txt` (T_ee_cam) | Once |
| Every time the paper is placed | Canvas detection `aruco_eih.py --robross` | `canvas_calibration_eih.yaml` | Before every painting session |
| Offline | Generate paths | `painting_paths.json` | When content changes |
| Execution | `ros2 launch ... paint.launch.py` | The arm paints | Every painting session |

## One-Sentence Summary

**Calibration solves "camera vs. arm", canvas detection solves "paper vs. robot",
and the path file solves "what to draw"** — together, the paper can be placed
anywhere, at any angle, and still be painted correctly.
