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

    # ── 实时 TCP 坐标刷新 (限频) ──────────────────────────────
    last_tcp_fetch = 0.0
    tcp_fetch_interval = 0.5  # 秒
    current_base_xyz = None   # 当前实时 TCP (用于显示)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        display = frame.copy()
        h, w = display.shape[:2]

        ids, corners, poses = detector.detect(display, cmat, dist)
        marker_found = len(ids) > 0

        # 限频实时读取机器人 TCP 坐标 (用于显示，不存储)
        now = time.time()
        if robot and now - last_tcp_fetch > tcp_fetch_interval:
            pose = robot.get_tcp_pose()
            if pose is not None:
                current_base_xyz = np.array(pose[:3])
            last_tcp_fetch = now

        # 当前检测到的 ArUco 信息
        current_cam_xyz = None
        current_mid = None
        current_rvec = None
        if marker_found and len(poses) > 0 and poses[0][1] is not None:
            current_mid = ids[0]
            current_rvec = poses[0][0]
            current_cam_xyz = poses[0][1].flatten()
            cv2.drawFrameAxes(display, cmat, dist,
                              current_rvec, current_cam_xyz,
                              args.marker_size * 0.6)

        # ══════════════════════════════════════════════════════════
        #  信息叠加 (类 aubo_writer 风格, 左上角简洁显示)
        # ══════════════════════════════════════════════════════════
        n_history = len(points_base)

        # 检测状态
        if current_mid is not None:
            status = f"✓  ID:{current_mid}"
            status_color = (0, 220, 80)
        else:
            status = "✗  未检测到"
            status_color = (0, 0, 230)

        # 检测到的所有 ID
        detected_ids_str = ""
        if marker_found:
            detected_ids_str = f"  检测到 ID: {sorted(ids)}"

        # 第 1 行: 采集数 + 检测状态 + 检测到的 ID
        line1 = f"已采集: {n_history} 组  |  检测: {status}{detected_ids_str}"
        cv2.putText(display, line1, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 2)

        # 第 2 行: 相机坐标
        if current_cam_xyz is not None:
            txt_cam = (f"Camera:  ({current_cam_xyz[0]:.3f}, "
                       f"{current_cam_xyz[1]:.3f}, "
                       f"{current_cam_xyz[2]:.3f})  m")
            cv2.putText(display, txt_cam, (10, 52),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 80), 1)

        # 第 3 行: 基座坐标 (实时)
        if robot and current_base_xyz is not None:
            txt_base = (f"Base:    ({current_base_xyz[0]:.3f}, "
                        f"{current_base_xyz[1]:.3f}, "
                        f"{current_base_xyz[2]:.3f})  m")
            cv2.putText(display, txt_base, (10, 74),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 200, 255), 1)
            cv2.putText(display, "●", (330, 74),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1)
        elif not robot and current_cam_xyz is not None:
            cv2.putText(display, "Base:    (手动输入)", (10, 74),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

        # 已采集点对列表 (右上角紧凑显示)
        if n_history > 0:
            list_x = w - 320
            # 浅色背景短条
            list_h = min(n_history, 5) * 18 + 28
            cv2.rectangle(display, (list_x, 8), (list_x + 310, 8 + list_h),
                          (20, 20, 20), -1)
            list_roi = display[8:8 + list_h, list_x:list_x + 310]
            cv2.addWeighted(list_roi, 0.7, np.full_like(list_roi, 20), 0.3, 0, list_roi)

            cv2.putText(display, f"─ 采集点: {n_history} 对 ─",
                        (list_x + 8, list_y := 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
            for i in range(min(n_history, 5)):
                pc = points_cam[i]
                pb = points_base[i]
                ry = list_y + 14 + i * 18
                cv2.putText(display,
                            f"#{i+1:02d} cam({pc[0]:.3f},{pc[1]:.3f},{pc[2]:.3f})",
                            (list_x + 10, ry),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (160, 220, 160), 1)
                cv2.putText(display,
                            f"    base({pb[0]:.3f},{pb[1]:.3f},{pb[2]:.3f})",
                            (list_x + 10, ry + 13),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (160, 190, 230), 1)
            if n_history > 5:
                cv2.putText(display, f"  ... {n_history - 5} 对隐藏",
                            (list_x + 10, list_y + 14 + 5 * 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

        # 底部操作提示
        cv2.putText(display, "[Space] 采集    [c] 完成    [q] 退出",
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (200, 200, 200), 1)

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
                current_base_xyz = base_xyz  # 同步实时显示
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
