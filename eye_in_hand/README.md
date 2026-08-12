# 臂上相机画布定位

本文是中文快速说明。英文版 [`README_EN.md`](README_EN.md) 是规范文档；
参数、限制或流程不一致时，以英文版为准。

## 用途

臂上 D405 通过 AX=XB 标定相机与控制器当前 TCP 的关系，并利用四个 ArUco
标记定位画布。历史名称 `T_ee_cam` 保留在文件和代码中，但控制器调用实际读取
TCP 位姿；更改相机安装或控制器 TCP 定义后必须重新标定。

## 基本流程

1. 按 [`README_EN.md`](README_EN.md) 完成相机内参和 eye-in-hand 标定。
2. 操作者把机械臂移到画布上方；检测程序只读取位姿，不会移动机械臂。
3. 运行 `aruco_eih.py --robross` 生成画布原点和方向 YAML。
4. 核对程序报告的画布宽高是否与路径文件一致。
5. 按 [`../docs/hardware_run_guide.md`](../docs/hardware_run_guide.md) 完成画布、
   dry-run、悬空、测试线和曲线验证后，才允许接触运行。

```bash
python3 eye_in_hand/aruco_eih.py \
  --robot-ip <控制器-IP> \
  --camera-id <相机-ID> \
  --robross
```

生成的 YAML 只是定位结果，不是接触运行许可。不要从相机检测直接启动真机画画，
也不要依赖 `paint.launch.py` 的 RViz 默认参数。

完整标定、质量指标、标记尺寸、相机模式和故障排查见英文规范文档
[`README_EN.md`](README_EN.md)。
