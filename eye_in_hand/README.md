# 臂上相机 (Eye-in-Hand) — 画布检测

把 D405 装在机械臂末端，让相机**靠近纸面**检测 ArUco 标记。
D405 是短距相机（最佳 7–30cm），臂上安装正好利用它的近距精度，
解决固定相机在 50cm 外识别不了 20mm 标记的问题。

```
      ┌──────────┐
      │  D405    │  ← 固定在机械臂末端
      └────┬─────┘
           │  30~40cm
      ┌────┴─────┐
      │  A4 纸   │
      └──────────┘
```

## 与 handeye_calibration 的区别

| | 固定相机 (handeye_calibration/) | 臂上相机 (eye_in_hand/) |
|---|---|---|
| 相机位置 | 桌面支架，固定不动 | 机械臂末端，随臂移动 |
| 标定目标 | `T_base_cam` (固定矩阵) | `T_ee_cam` (相机在末端中) |
| 标定方法 | 点对 SVD (Kabsch) | **AX=XB** (Park-Martin) |
| 坐标变换 | `T_base_cam` 固定 | `T_base_cam = T_base_ee × T_ee_cam` (动态) |
| 检测距离 | 50cm 以上容易失效 | 20–40cm，稳定 |
| 视野 | 大，一次看全纸 | 小，需要臂移到纸上方 |

## 标定 (calibrate_eih.py)

目的：求解 `T_ee_cam` — 相机坐标系在机械臂末端坐标系中的位姿。

```
A_i = T_base_ee_i        (机械臂读取)
B_i = T_cam_marker_i     (摄像头 solvePnP)
A_i X B_i = 常数         (标记在基座中固定不动)
→ AX = XB，X = T_ee_cam
```

### 操作流程

> **★ 不需要笔尖碰标记！** 与固定相机版本不同，eye-in-hand 标定
> 只需要标记固定不动、机械臂摆出不同位姿让相机看清标记即可。
> 笔尖离标记多远都无所谓，甚至不需要装笔。

1. **ArUco 标记固定在桌面上，标定过程中不能移动!**
2. 相机装在机械臂末端，机械臂切到拖拽模式
3. 拖到不同位姿（角度、远近都要变化），使标记在画面中，**保持距离 10~25cm**
   （D405 最佳 7–30cm，太近会失焦、太远又回到 50cm 识别问题）
4. 每个位姿按 `[Space]` 记录（自动读 TCP 位姿 + 标记位姿）
5. **每次采集后程序立即显示实时质量报告**——看到「旋转轴分布合理 +
   旋转误差 < 2°」就可以按 `[c]` 完成（一般 6~8 组）
6. 按 `[c]` 完成 → 得到 `eye_in_hand_calib.txt`

```bash
# 预览流程
python3 eye_in_hand/calibrate_eih.py --robot-ip 192.168.1.100 --dry-run

# 实际标定
python3 eye_in_hand/calibrate_eih.py \
  --robot-ip 192.168.1.100 \
  --camera-id <ID>
```

> **注意：eye-in-hand 不能使用缓存！** A（TCP）和 B（标记位姿）必须是
> **同一时刻**读取的。标记被机械臂遮挡时按 Space 会被拒绝——请调整位姿
> 让标记重新可见再采集。固定相机版本可以用缓存，臂上相机不行。

### 实时质量报告（每次采集后自动计算）

采集满 3 组后，每次按 `[Space]` 程序都会立即重新求解并打印：

```
  ┌─ 实时质量 (4 组)
  │  旋转轴分布: x=0.54 y=0.40 z=0.06  ✓ 合理
  │  一致性误差: 位置 0.0 mm, 旋转 0.1°
  └─
```

| 指标 | 健康信号 | 问题信号 |
|---|---|---|
| 旋转轴分布 | x/y/z 三个方向都有分量（如 0.4/0.3/0.3） | 第一项 > 0.85 = **退化**，旋转解不可信 |
| 一致性误差 | 旋转 < 2° | > 5° 会提示「换个朝向再采一组」 |

**旋转轴退化是最常见的标定失败原因**：位姿数量再多，如果相机始终几乎
正对标记（只平移、只绕一个方向转），绕其他轴的旋转信息就缺失，解出的
旋转误差可以到十几度。解决：故意让相机倾斜、旋转、俯仰，三个方向的
奇异值都上来了再继续。

### 数据保存与离线分析

按 `[d]` 可把当前采集的 A/B 数据保存为 `eye_in_hand_calib_data.npz`，
配合离线分析工具诊断：

```bash
python3 eye_in_hand/analyze_eih_data.py eye_in_hand_calib_data.npz
```

输出包括旋转轴分布、位姿覆盖范围（距离/角度变化）和一致性误差，
帮助判断是数据退化、solvePnP 噪声还是 TCP 读数问题。

### 判断标定质量

程序会验证「标记在基座坐标系中的位姿一致性」：

| 平均位置误差 | 平均角度误差 | 评价 |
|---|---|---|
| < 5 mm | < 1° | 优秀 |
| 5–15 mm | 1–3° | 可用 |
| > 15 mm 或 > 5° | — | 重新做，检查位姿分布 |

## 画布检测 (aruco_eih.py)

标定完成后，把纸放在桌面上（4 角贴 ArUco），机械臂移到纸上方：

```bash
# 查看摄像头 ID
python3 eye_in_hand/aruco_eih.py --list-cameras

# 预览 (不保存)
python3 eye_in_hand/aruco_eih.py \
  --robot-ip 192.168.1.100 \
  --camera-id <ID> \
  --dry-run

# 生成 RobRoss 画布标定 YAML
python3 eye_in_hand/aruco_eih.py \
  --robot-ip 192.168.1.100 \
  --camera-id <ID> \
  --robross
```

每次检测时实时读取机械臂 TCP，动态计算：

```
T_base_cam = T_base_ee × T_ee_cam
```

生成的 YAML 和固定相机版本格式完全一样（`canvas_origin_xyz` +
`canvas_quat_xyzw`），可以直接喂给 `paint.launch.py`。

## 文件说明

| 文件 | 用途 |
|---|---|
| `README_EN.md` | 英文说明（与本文对应） |
| `calibrate_eih.py` | AX=XB 标定 → `eye_in_hand_calib.txt` |
| `analyze_eih_data.py` | 离线分析采集数据（npz）→ 诊断标定质量 |
| `aruco_eih.py` | 实时检测画布 → RobRoss 画布 YAML |
| `eih_common.py` | 本目录公共模块：检测器、标定加载、画布计算、机械臂通信 |
| `calibrate_camera.py` | 相机内参标定（本目录独立） |
| `read_realsense_intrinsics.py` | 读取 D405 出厂内参 |
| `markers/` | 可打印的 ArUco 标记（4 角 + 标定用） |
| `chessboard.png` | 棋盘格（相机标定用） |
| `camera_calib.json` / `_d405.json` / `_d435.json` | 相机内参文件 |
| `requirements.txt` | Python 依赖 |
| `eye_in_hand_calibration_guide.md` / `_EN.md` | 标定操作指南（中/英） |
| `start_painting_eih.sh` | 一键启动：臂上相机检测 → ROS 2 画画 |
| `eye_in_hand_calib.txt` | 标定输出（4×4 矩阵 `T_ee_cam`） |

本目录**完全自包含**：检测器、相机内参加载、绘图区域计算、机械臂通信
（JSON-RPC）和画布输出逻辑都在 `eih_common.py` 中，不依赖
`handeye_calibration/`，单独复制本文件夹即可运行。

### 一键启动（标定完成后）

```bash
# 从本目录运行
cd eye_in_hand/
./start_painting_eih.sh \
  --robot-ip 192.168.32.101 \
  --camera-id <ID> \
  --paths-file /path/to/painting_paths.json \
  --calibration-file /path/to/hardware_a4.yaml
```

与固定相机版本（`handeye_calibration/start_painting.sh`）的区别：
必须提供 `--robot-ip`（臂上相机需实时读取末端位姿），且使用
`eye_in_hand_calib.txt`（T_ee_cam）而非 `handeye_calib.txt`。

## 备注

- 相机标定文件在本目录内（`camera_calib.json` 等），与固定相机版本互不影响
- 机械臂通信走 JSON-RPC（与主项目一致，无 pyaubo_sdk 依赖）
- 臂上相机只解决「相机靠近纸面」的问题；画画时机械臂仍需回到固定相机
  方案或由用户引导——当前版本只输出画布标定，不直接控制画画路径
