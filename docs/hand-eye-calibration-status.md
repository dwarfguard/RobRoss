# 手眼标定状态说明 / Hand-Eye Calibration Status

**Status:** Issue draft — 可直接贴到 GitHub
**Last updated:** 2026-08-06

---

# 中文版 (Issue 草稿)

## 手眼标定状态说明：eye-in-hand 标定已可用，附后续标定操作

### 背景

项目目前有两条相机标定路径：

| 方案 | 目录 | 相机位置 | 标定目标 |
|---|---|---|---|
| 固定相机 | `handeye_calibration/` | 桌面支架 | `T_base_cam`（相机→基座固定矩阵） |
| 臂上相机 | `eye_in_hand/` | 机械臂末端 | `T_ee_cam`（相机→末端固定矩阵） |

### 当前标定数据情况

**✅ 臂上相机（`eye_in_hand/`）— 已完成，可直接使用**

已提交的文件：
- `eye_in_hand/eye_in_hand_calib.txt` — `T_ee_cam` 4×4 变换矩阵
- `eye_in_hand/eye_in_hand_calib_data.npz` — 标定采集原始数据（可用 `analyze_eih_data.py` 离线复现）

实测质量指标（`analyze_eih_data.py` 输出，6 组位姿）：

| 指标 | 数值 | 评价标准 |
|---|---|---|
| 位置一致性误差 | 平均 2.23 mm / 最大 3.63 mm | 优秀：< 5 mm |
| 旋转一致性误差 | 平均 0.74° / 最大 1.17° | 优秀：< 1° |
| 旋转轴分布 | x=0.59 y=0.29 z=0.13 | 合理，未退化 |
| 采集距离 | 245 ~ 305 mm | 略高于推荐的 100–250mm 区间，但结果仍可用 |

结论：达到“优秀”标准，`aruco_eih.py` 会自动加载该文件做画布检测。

此前标定一直失败的两个根因（commit `8eeedd9` 已修复）：

1. **相机分辨率不匹配**：D405 走 UVC 时以 640×480（传感器裁剪）输出，而内参文件是 1280×720 的，导致系统性深度/位置误差。现改为优先用 pyrealsense2 SDK 强制 1280×720 并读取该流实时内参（含出厂畸变），UVC 仅作回退。
2. **标定标记尺寸不匹配**：标定用大标记黑边约 48mm，代码默认 20mm，导致 solvePnP 距离低估。现在 `calibrate_eih.py` 会自动反推黑边尺寸并提示正确的 `--marker-size`。

**⚠️ 固定相机（`handeye_calibration/`）— 尚未完成**

`handeye_calibration/handeye_calib.txt` 还没有生成/提交。如果需要固定相机方案（大视野、一次看全整张纸），需要另行完成标定；目前只有臂上相机方案有可用数据。

### 后续如何做标定

**臂上相机标定（推荐，当前唯一有数据的路径）：**

前置条件：
- D405 固定在机械臂末端；机械臂开机并能切换拖拽模式
- 打印大 ArUco 标记（黑边约 48mm）贴在硬纸板上，**整个标定过程固定不动**
- 安装 pyrealsense2（`pip install pyrealsense2`），确保程序走 SDK 路径

```bash
cd eye_in_hand/
python3 calibrate_eih.py \
  --robot-ip 192.168.32.101 \
  --camera-id <ID> \
  --marker-size 0.048
```

操作步骤：

1. 机械臂切到拖拽模式，拖到标记上方，让标记在画面中清晰可见
2. 距离保持 **10~25cm**；每个位姿都要改变角度和远近（俯仰/侧倾/旋转都要有）
3. 每组摆好后停 1 秒（机械臂完全静止）再按 `[Space]` 记录
4. 采集 6~8 组；每次采集后自动打印实时质量报告，看到“旋转轴分布合理 + 旋转误差 < 2°”即可按 `[c]` 完成
5. 结果自动保存为 `eye_in_hand_calib.txt`

关键提醒：

- **两个阶段的标记尺寸不同，不要混用**：标定用大标记 `--marker-size 0.048`；画布检测用纸上 4 角的小标记（20mm，`aruco_eih.py` 默认 `0.02`，无需改）
- **臂上相机不能用缓存**：A（TCP 位姿）和 B（标记位姿）必须同一时刻读取，标记被机械臂挡住时按 Space 会被拒绝，需要换位姿让标记重新可见
- 采错一组按 `[Backspace]` 删除；中途按 `[d]` 保存数据，用 `analyze_eih_data.py` 离线诊断
- 常见错误信号：旋转轴分布第一项 > 0.85 = 位姿退化（只绕同一方向转），继续采也没用，必须换朝向

质量判断标准：

| 平均位置误差 | 平均角度误差 | 评价 |
|---|---|---|
| < 5 mm | < 1° | 优秀，直接使用 |
| 5–15 mm | 1–3° | 可用，可再采几组优化 |
| > 15 mm 或 > 5° | — | 重做，检查位姿分布 |

**固定相机标定（如果后续需要）：**

目前无数据。操作步骤见 `handeye_calibration/README.md` 和 `handeye_calibration_guide.md`：用点对 SVD（Kabsch），笔尖触碰标记中心，采集 4~6 组，输出 `handeye_calib.txt`。

### 相关文档

- `eye_in_hand/eye_in_hand_calibration_guide.md` / `_EN.md` — 臂上相机标定操作指南
- `eye_in_hand/README.md` / `README_EN.md` — 目录说明、命令、常见错误排查
- `handeye_calibration/README.md` — 固定相机方案说明

---

# English Version (Issue Draft)

## Hand-Eye Calibration Status: Eye-in-Hand Calibration Is Ready, Plus How to Re-Calibrate

### Background

The project has two camera-based calibration paths:

| Approach | Directory | Camera position | Calibration target |
|---|---|---|---|
| Fixed camera | `handeye_calibration/` | Desk mount | `T_base_cam` (fixed camera→base transform) |
| Eye-in-hand | `eye_in_hand/` | Robot arm end-effector | `T_ee_cam` (camera in end-effector frame) |

### Current Calibration Data Status

**✅ Eye-in-hand (`eye_in_hand/`) — Done, ready to use**

Committed files:
- `eye_in_hand/eye_in_hand_calib.txt` — `T_ee_cam` 4×4 transform matrix
- `eye_in_hand/eye_in_hand_calib_data.npz` — raw captured data (reproducible offline via `analyze_eih_data.py`)

Measured quality metrics (`analyze_eih_data.py` output, 6 poses):

| Metric | Value | Rating criterion |
|---|---|---|
| Position consistency error | mean 2.23 mm / max 3.63 mm | Excellent: < 5 mm |
| Rotation consistency error | mean 0.74° / max 1.17° | Excellent: < 1° |
| Rotation axis distribution | x=0.59 y=0.29 z=0.13 | Healthy, not degenerate |
| Capture distance | 245–305 mm | Slightly beyond the recommended 100–250 mm range, but the result is still usable |

Conclusion: meets the "excellent" bar; `aruco_eih.py` loads this file automatically for canvas detection.

The two root causes that previously broke calibration (fixed in commit `8eeedd9`):

1. **Camera resolution mismatch**: over UVC the D405 outputs 640×480 (sensor crop) while the intrinsics file is for 1280×720, causing systematic depth/position error. The code now prefers the pyrealsense2 SDK, which forces 1280×720 and reads the live intrinsics (including factory distortion) for that stream; UVC is only a fallback.
2. **Calibration marker size mismatch**: the large calibration marker's black border is ~48 mm while the code default was 20 mm, so solvePnP underestimated distances. `calibrate_eih.py` now infers the black-border size automatically and suggests the correct `--marker-size`.

**⚠️ Fixed camera (`handeye_calibration/`) — Not done yet**

`handeye_calibration/handeye_calib.txt` has not been generated/committed. If the fixed-camera approach is needed (large field of view, sees the whole sheet at once), calibration must be performed separately; only the eye-in-hand path currently has usable data.

### How to Re-Calibrate

**Eye-in-hand calibration (recommended; the only path with data so far):**

Prerequisites:
- D405 mounted on the robot end-effector; robot powered on and able to switch to drag (freedrive) mode
- Print a large ArUco marker (black border ~48 mm) on cardboard; **it must stay fixed during the whole process**
- Install pyrealsense2 (`pip install pyrealsense2`) so the program uses the SDK path

```bash
cd eye_in_hand/
python3 calibrate_eih.py \
  --robot-ip 192.168.32.101 \
  --camera-id <ID> \
  --marker-size 0.048
```

Procedure:

1. Switch the arm to drag mode and move it above the marker so the marker is clearly visible in the frame.
2. Keep the distance at **10–25 cm**; vary both angle and distance for every pose (pitch, roll, and yaw rotation).
3. Let the arm settle for 1 second (completely still) before pressing `[Space]` to record.
4. Capture 6–8 poses; after each capture the program prints a live quality report. Once it shows "rotation axis distribution healthy + rotation error < 2°", press `[c]` to finish.
5. The result is saved automatically as `eye_in_hand_calib.txt`.

Key reminders:

- **Do not mix up the two marker sizes**: calibration uses the large marker (`--marker-size 0.048`); canvas detection uses the four small corner markers on the paper (20 mm — the `aruco_eih.py` default of `0.02`, no change needed)
- **No caching for eye-in-hand**: A (TCP pose) and B (marker pose) must be read at the same instant. Pressing `[Space]` while the marker is occluded by the arm is rejected — reposition so the marker is visible again
- Press `[Backspace]` to delete the last capture; press `[d]` to save the data and diagnose offline with `analyze_eih_data.py`
- Common failure signal: first singular value of the rotation axis distribution > 0.85 = degenerate poses (rotating around a single axis); capturing more of the same is pointless — change the orientation

Quality criteria:

| Mean position error | Mean rotation error | Rating |
|---|---|---|
| < 5 mm | < 1° | Excellent, use as-is |
| 5–15 mm | 1–3° | Usable, a few more poses recommended |
| > 15 mm or > 5° | — | Redo; check pose distribution |

**Fixed-camera calibration (if needed later):**

No data yet. See `handeye_calibration/README.md` and `handeye_calibration_guide.md`: point-correspondence SVD (Kabsch), touch the marker center with the pen tip, capture 4–6 samples, output `handeye_calib.txt`.

### Related Docs

- `eye_in_hand/eye_in_hand_calibration_guide.md` / `_EN.md` — eye-in-hand calibration guide
- `eye_in_hand/README.md` / `README_EN.md` — directory docs, commands, common troubleshooting
- `handeye_calibration/README.md` — fixed-camera approach
