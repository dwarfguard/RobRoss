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
5. 至少 3 个位姿，建议 **6~8 个**
6. 按 `[c]` 完成 → 得到 `eye_in_hand_calib.txt`

```bash
# 预览流程
python3 eye_in_hand/calibrate_eih.py --robot-ip 192.168.1.100 --dry-run

# 实际标定
python3 eye_in_hand/calibrate_eih.py \
  --robot-ip 192.168.1.100 \
  --camera-id <ID>
```

标记被机械臂遮挡时，程序会缓存最后一次检测到的标记位姿（画面显示黄色 `◉`），
按 Space 时自动使用缓存——和固定相机版本的逻辑一致。

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
| `aruco_eih.py` | 实时检测画布 → RobRoss 画布 YAML |
| `calibrate_camera.py` | 相机内参标定（与固定相机版本相同） |
| `read_realsense_intrinsics.py` | 读取 D405 出厂内参 |
| `markers/` | 可打印的 ArUco 标记（4 角 + 标定用） |
| `chessboard.png` | 棋盘格（相机标定用） |
| `camera_calib.json` / `_d405.json` / `_d435.json` | 相机内参文件 |
| `requirements.txt` | Python 依赖 |
| `eye_in_hand_calibration_guide.md` / `_EN.md` | 标定操作指南（中/英） |
| `start_painting_eih.sh` | 一键启动：臂上相机检测 → ROS 2 画画 |
| `eye_in_hand_calib.txt` | 标定输出（4×4 矩阵 `T_ee_cam`） |

代码复用 `handeye_calibration/aruco_drawing_area.py` 中的检测器、相机内参
加载、绘图区域计算和画布输出逻辑。

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

- 相机标定文件复用 `handeye_calibration/camera_calib.json`
- 机械臂通信走 JSON-RPC（与主项目一致，无 pyaubo_sdk 依赖）
- 臂上相机只解决「相机靠近纸面」的问题；画画时机械臂仍需回到固定相机
  方案或由用户引导——当前版本只输出画布标定，不直接控制画画路径
