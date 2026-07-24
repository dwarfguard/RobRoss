# 手眼标定操作指南

## 用来干什么

把摄像头看到的坐标转换成机械臂知道的坐标。

```
摄像头看见 ArUco 标记 → [0.12, -0.05, 0.35] （相机坐标）
                      → ？                     （机械臂坐标）
```

**手眼标定**就是找出这个"？"，得到一个 4×4 矩阵 `T_base_cam`，存到 `handeye_calib.txt`。

---

## 准备工作

### 硬件

| 物品 | 说明 |
|------|------|
| D405 摄像头 | 固定好，不能动。对准绘图区域 |
| ArUco 标记 | 用 `--generate-markers` 生成并打印一个，贴到硬纸板上 |
| 机械臂 | 开机，能切换到**拖拽模式（手扶示教）** |
| 笔尖 | 装在机械臂末端，TCP 已标定好 |

### 确认文件

```bash
ls -la camera_calib.json    # 必须存在（存的是 D405 内参）
ls -la markers/             # 必须有 ArUco 标记图片
```

### 安装依赖

```bash
pip install opencv-python opencv-contrib-python numpy
```

### 测试通信

```bash
python3 aruco_drawing_area.py --test-comm --robot-ip 192.168.1.100
# 看到当前 TCP 位姿 → 通信正常
```

---

## 操作流程

### 第 1 步：运行标定程序

```bash
python3 calibrate_handeye.py --robot-ip 192.168.1.100
```

如果只是看流程：

```bash
python3 calibrate_handeye.py --robot-ip 192.168.1.100 --dry-run
```

### 第 2 步：采集点对（重复 5-6 次）

每次都做同样的事：

```
1. 把 ArUco 标记放在桌面上一个位置

2. 机械臂切到拖拽模式

3. 用手推机械臂，让笔尖精准碰到标记中心 ⬇️
         ┌──────┐
         │  ◉   │  ← 笔尖碰这里
         └──────┘
         ArUco 标记

4. 在键盘上按 [Space] 记录
   → 程序自动读两个值：
      摄像头：标记中心在相机坐标中的位置
      机械臂：TCP 在基座坐标中的位置
   → 屏幕显示 "第 1 对" "第 2 对" ...
```

**采集技巧（决定标定质量）：**

| 要求 | 原因 |
|------|------|
| 5-6 个点 | 太少算不准 |
| 高低前后都分布 | 点集中在同一平面会缺 Z 方向信息 |
| 不要都在一条线上 | 三点共线解不出旋转 |
| 覆盖整个摄像头视野 | 保证工作区域内都准确 |

好的分布示例：

```
  z ↑          · (高, 远)
    |    · (中, 左)     · (中, 右)
    |       · (低, 近)
    +——————————————→ x/y
```

### 第 3 步：完成标定

按 `c` 键。

程序自动计算并显示：

```
  ┌─ 结果: T_base_cam
  │  R: [[ 0.99,  0.01,  0.05],     ← 旋转矩阵 (3×3)
  │      [-0.01,  0.99, -0.02],
  │      [-0.05,  0.02,  0.99]]
  │  t: [0.45, -0.12, 0.38]         ← 平移向量 (米)
  │  平均残差: 1.23 mm               ← 这个数越小越好
  │  最大残差: 2.10 mm
  └─
```

**T_base_cam 是什么？**

是一个 4×4 齐次变换矩阵，把相机坐标转为机械臂基座坐标：

```
p_base = T_base_cam × p_cam
  ↑                        ↑
基座坐标                 相机坐标
```

```
        ┌                 ┐
        │ R00 R01 R02  tx │
T_base_cam = │ R10 R11 R12  ty │
        │ R20 R21 R22  tz │
        │  0   0   0   1 │
        └                 ┘
```

| 部分 | 含义 |
|------|------|
| **R** | 旋转 — 相机朝向在基座坐标系中的描述 |
| **t** | 平移 — 相机原点在基座坐标系中的位置 |
| **RPY** | 欧拉角，R 的可读形式 |

保存文件 `handeye_calib.txt` 就是这 4×4 矩阵：

```
R00 R01 R02 tx
R10 R11 R12 ty
R20 R21 R22 tz
0   0   0   1
```

**判断标定质量：**

| 残差 | 评价 |
|------|------|
| < 2 mm | 优秀，可以直接用 |
| 2-5 mm | 可用，可以再采几个点优化 |
| > 5 mm | 重新做，检查笔尖是否对准标记中心 |

按 `y` 保存，得到 `handeye_calib.txt`。

---

## 坐标变换全链路

手眼标定得到 `T_base_cam` 后，整个画画流程的坐标变换链路如下：

```
摄像头看见纸面点 P
    │
    ▼
P_cam  (相机坐标)
    │
    │  T_base_cam  (手眼标定得到的 4×4 矩阵)
    ▼
P_tcp  ── 笔尖目标位置 (基座坐标)
    │
    ├── [JSON-RPC 直连管道]
    │   moveLine([P_tcp, roll, pitch, yaw])
    │   └→ AUBO 控制器用 TCP 偏移把笔尖目标转成法兰轨迹
    │
    └── [ROS 2 / MoveIt 管道]
        T_ee = T_tip × tool_offset_inv_
        └→ tool_offset_xyz 描述笔尖在法兰坐标系中的位置
        └→ MoveIt 规划法兰运动
```

**T_base_cam 映射到笔尖，不是法兰。** 因为标定时 `get_tcp_pose()` 读的就是笔尖坐标。
然后再由 AUBO 控制器（JSON-RPC 管道）或 `tool_offset_inv_`（ROS 2 管道）把笔尖目标转成法兰中心轨迹。

> AUBO 示教器上配的 TCP 偏移必须与 `config/hardware_a4.yaml` 中的 `tool_offset_xyz` 一致。
> 当前值: `[0.0595, 0, 0.0514]` 米。

---

## 常见问题

### Q: 笔尖碰不到标记中心？

- 换大一点的标记（改 `--marker-size` 参数）
- 标记贴在硬纸板而不是软纸上
- 笔尖太粗的话，用 ArUco 标记的一个角代替中心

### Q: 标定完残差很大？

- 检查每个点是否真的对准了（离线检查：看图片回放）
- 点的分布是否太集中
- 机械臂 TCP 标定是否准确

### Q: 没有机械臂也能标定吗？

可以，用 `--manual` 模式手动输入基座坐标：

```bash
python3 calibrate_handeye.py --manual
```

但需要手动测量点在机械臂基座中的位置，精度有限。

### Q: 摄像头动过了怎么办？

**重新标定。** 摄像头位置一变，`handeye_calib.txt` 就作废了。

---

## 标定完成后

用 `aruco_drawing_area.py` 验证：

```bash
# 方式 A：直接控制机械臂
python3 aruco_drawing_area.py --camera-id 2 --robot-ip 192.168.1.100 --start-from 0

# 方式 B：生成画布标定文件（给 RobRoss ROS 2 用）
python3 aruco_drawing_area.py --camera-id 2 --robross --robross-output canvas_calibration.yaml
```

看到机械臂准确移动到绘图区域的角点 → 标定成功。

---

## 一句话总结

```
运行 calibrate_handeye.py → 碰 5 次标记按 Space → 按 c 完成
```
