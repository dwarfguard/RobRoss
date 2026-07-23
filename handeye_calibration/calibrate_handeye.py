#!/usr/bin/env python3
"""
手眼标定工具 (Eye-to-Hand)
===========================

目的: 求解 T_base_cam — 将摄像头坐标系变换到机械臂基座坐标系。

操作流程:
  1. 固定摄像头，标记放在桌面
  2. 机械臂切换到**拖拽/手扶示教**模式
  3. 用笔尖/末端 TCP 碰标记中心
  4. 按 [Space] 记录——程序自动从机械臂读 TCP 坐标 + 摄像头检测标记
  5. 标记移动 4~6 个不同位置，重复 3~4
  6. 按 [c] 完成标定

不需要手动输入坐标，程序通过 JSON-RPC 直接从机械臂读。
如果不用机械臂，也可以用 --manual 手动输入。

用法:
  python3 calibrate_handeye.py --robot-ip 192.168.1.100
  python3 calibrate_handeye.py --robot-ip 192.168.1.100 --dry-run   # 看流程
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import json
import os
import sys
import argparse
import time
sys.path.insert(0, os.path.dirname(__file__))
from aruco_drawing_area import (
    ArucoDetector, CameraCalib, JsonRpcBackend, SdkBackend, SDK_AVAILABLE,
    create_robot_backend
)


def load_camera_calib(path="camera_calib.json"):
    if not os.path.exists(path):
        print(f"[⚠] 未找到 {path}")
        return None, None
    calib = CameraCalib.load(path)
    print(f"[✓] 加载相机内参: fx={calib.camera_matrix[0,0]:.1f} fy={calib.camera_matrix[1,1]:.1f}")
    return calib.camera_matrix, calib.dist_coeffs


def rotation_matrix_to_rpy(R):
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        return np.array([np.arctan2(R[2, 1], R[2, 2]),
                         np.arctan2(-R[2, 0], sy),
                         np.arctan2(R[1, 0], R[0, 0])])
    return np.array([np.arctan2(-R[1, 2], R[1, 1]),
                     np.arctan2(-R[2, 0], sy), 0.0])


def main():
    parser = argparse.ArgumentParser(description="手眼标定工具 (Eye-to-Hand)")
    parser.add_argument("--camera-id", type=int, default=0, help="摄像头 ID")
    parser.add_argument("--camera-calib", default="camera_calib.json",
                        help="相机标定文件")
    parser.add_argument("--aruco-dict", default="4X4_50", help="ArUco 字典")
    parser.add_argument("--marker-size", type=float, default=0.035,
                        help="标记边长/米")
    parser.add_argument("--output", default="handeye_calib.txt",
                        help="输出文件 (默认 handeye_calib.txt)")
    parser.add_argument("--robot-ip", help="机械臂 IP (自动读取 TCP 坐标)")
    parser.add_argument("--robot-port", type=int, default=8899)
    parser.add_argument("--manual", action="store_true",
                        help="手动输入基座坐标 (不用机械臂)")
    parser.add_argument("--dry-run", action="store_true", help="看流程")
    args = parser.parse_args()

    if args.dry_run:
        print("\n" + "=" * 55)
        print("  手眼标定操作步骤")
        print("=" * 55)
        print()
        print("  1. 固定好摄像头 (确定不动了)")
        print("  2. 打印 ArUco 标记，贴在硬纸板上")
        print("  3. 标记放桌面，机械臂切到拖拽模式")
        print("  4. TCP 触碰标记中心，按 [Space] 记录")
        print("     → 自动读取机械臂 TCP 坐标 + 摄像头坐标")
        print("  5. 标记移到不同位置 (高低前后都分布)")
        print("     重复 4~6 次")
        print("  6. 按 [c] 完成 → 得到 handeye_calib.txt")
        print()
        print("  运行: python3 calibrate_handeye.py --robot-ip 192.168.1.100")
        print("=" * 55)
        return

    if not args.robot_ip and not args.manual:
        print("[✗] 需要 --robot-ip <IP> (自动读取) 或 --manual (手动输入)")
        return

    cmat, dist = load_camera_calib(args.camera_calib)
    if cmat is None:
        return

    # 连接机械臂
    robot = None
    if args.robot_ip:
        robot = create_robot_backend(args.robot_ip, args.robot_port)

    # ArUco 检测器
    detector = ArucoDetector(args.aruco_dict, args.marker_size, target_ids=None)

    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        print(f"[✗] 无法打开摄像头 ID={args.camera_id}")
        return

    points_cam = []      # 相机坐标
    points_base = []     # 基座坐标
    print("\n" + "=" * 55)
    print("  手眼标定")
    print("=" * 55)
    print()
    print("  操作:")
    print(f"    [Space]  采集 — 触碰后按一下，{'自动读TCP' if robot else '手动输入'}")
    print("    [c]      标定完成并保存")
    print("    [q]      退出")
    print()

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        display = frame.copy()

        ids, corners, poses = detector.detect(display, cmat, dist)
        marker_found = len(ids) > 0

        if marker_found and len(poses) > 0 and poses[0][1] is not None:
            mid = ids[0]
            rvec, tvec = poses[0]
            cam_xyz = tvec.flatten()
            label = f"ID:{mid}  cam=({cam_xyz[0]:.3f}, {cam_xyz[1]:.3f}, {cam_xyz[2]:.3f})"
            cv2.putText(display, label, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.drawFrameAxes(display, cmat, dist, rvec, tvec, args.marker_size * 0.6)

        info = f"已采集: {len(points_base)} 对"
        cv2.putText(display, info, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        cv2.putText(display, "[Space]采集 [c]完成 [q]退出",
                    (10, display.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.imshow("Hand-Eye Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        elif key == 32 and marker_found and len(poses) > 0 and poses[0][1] is not None:
            cam_xyz = poses[0][1].flatten()
            mid = ids[0]

            if robot:
                # 从机械臂自动读取当前 TCP 坐标
                pose = robot.get_tcp_pose()
                if pose is None:
                    print("\n  [✗] 读取 TCP 失败，再试一次")
                    continue
                base_xyz = np.array(pose[:3])
                print(f"\n  ID:{mid}  相机: ({cam_xyz[0]:.4f}, {cam_xyz[1]:.4f}, {cam_xyz[2]:.4f})")
                print(f"           基座: ({base_xyz[0]:.4f}, {base_xyz[1]:.4f}, {base_xyz[2]:.4f})  ← 从机械臂自动读取")
                points_cam.append(cam_xyz.copy())
                points_base.append(base_xyz)
                print(f"  [✓] 第 {len(points_base)} 对")
            else:
                print(f"\n  ID:{mid}  相机: ({cam_xyz[0]:.4f}, {cam_xyz[1]:.4f}, {cam_xyz[2]:.4f})")
                inp = input("  输入基座坐标 (x,y,z): ")
                try:
                    base_xyz = np.array([float(v.strip()) for v in inp.split(",")])
                    if len(base_xyz) != 3:
                        print("  [✗] 需要 3 个值")
                        continue
                    points_cam.append(cam_xyz.copy())
                    points_base.append(base_xyz)
                    print(f"  [✓] 第 {len(points_base)} 对")
                except ValueError:
                    print("  [✗] 格式错误: x,y,z")

        elif key == ord("c"):
            if len(points_base) < 3:
                print(f"\n  [✗] 至少 3 个点，当前 {len(points_base)}")
                continue

            A = np.array(points_cam).T
            B = np.array(points_base).T
            ca = A.mean(axis=1, keepdims=True)
            cb = B.mean(axis=1, keepdims=True)
            Ac = A - ca
            Bc = B - cb
            U, _, Vt = np.linalg.svd(Ac @ Bc.T)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[-1] *= -1
                R = Vt.T @ U.T
            t = cb - R @ ca
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t.flatten()

            errors = [np.linalg.norm(
                (T @ np.append(points_cam[i], 1.0))[:3] - points_base[i])
                for i in range(len(points_base))]
            rpy = np.degrees(rotation_matrix_to_rpy(R))

            print(f"\n  ┌─ 结果: T_base_cam")
            print(f"  │  R:")
            for row in R:
                print(f"  │    [{row[0]:.6f}, {row[1]:.6f}, {row[2]:.6f}]")
            print(f"  │  t: [{t[0][0]:.4f}, {t[1][0]:.4f}, {t[2][0]:.4f}]")
            print(f"  │  RPY: {rpy[0]:.1f}°, {rpy[1]:.1f}°, {rpy[2]:.1f}°")
            print(f"  │  平均残差: {np.mean(errors)*1000:.2f} mm")
            print(f"  │  最大残差: {np.max(errors)*1000:.2f} mm")
            print(f"  └─")

            if np.max(errors) * 1000 > 15:
                inp = input("  残差较大 (>15mm)，仍然保存? (y/n): ")
                if inp.lower() != "y":
                    continue

            np.savetxt(args.output, T, fmt="%.6f")
            print(f"  [✓] 已保存: {args.output}")
            print(f"      aruco_drawing_area.py 会自动加载")
            break

    cap.release()
    cv2.destroyAllWindows()
    if robot:
        robot.disconnect()


if __name__ == "__main__":
    main()
