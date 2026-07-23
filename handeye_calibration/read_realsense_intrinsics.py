#!/usr/bin/env python3
"""
Intel RealSense 出厂内参读取工具 (Linux)

用法:
  1. pip install pyrealsense2
  2. python3 read_realsense_intrinsics.py
  3. 得到 camera_calib.json → 拷回项目目录
"""

import json
import sys


def main():
    try:
        import pyrealsense2 as rs
    except ModuleNotFoundError:
        print("[✗] 需要 pyrealsense2: pip install pyrealsense2")
        sys.exit(1)

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    print("[i] 启动相机...")
    profile = pipe.start(cfg)

    # 跳过热启动帧
    for _ in range(10):
        pipe.wait_for_frames()

    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    data = {
        "camera_matrix": [
            [round(intr.fx, 4), 0.0,               round(intr.ppx, 4)],
            [0.0,              round(intr.fy, 4),  round(intr.ppy, 4)],
            [0.0,              0.0,                1.0],
        ],
        "dist_coeffs": [[0.0, 0.0, 0.0, 0.0, 0.0]],
        "img_size": [intr.width, intr.height],
    }

    pipe.stop()

    with open("camera_calib.json", "w") as f:
        json.dump(data, f, indent=2)

    print(f"\n[✓] camera_calib.json 已生成")
    print(f"\n{'='*50}")
    print(json.dumps(data, indent=2, ensure_ascii=False))
    print(f"{'='*50}")
    print(f"\n  fx={intr.fx:.2f}  fy={intr.fy:.2f}")
    print(f"  cx={intr.ppx:.2f}  cy={intr.ppy:.2f}")
    print(f"  分辨率: {intr.width}×{intr.height}")
    print(f"  畸变系数: 全零 (RealSense 硬件校正)")
    print(f"\n拷贝 camera_calib.json 到 aubo-aruco 目录即可")


if __name__ == "__main__":
    main()
