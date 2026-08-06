#!/usr/bin/env python3
"""
臂上相机 (Eye-in-Hand) 绘制区域检测 → RobRoss 画布标定
========================================================

相机固定在机械臂末端，机械臂移动到纸面上方，相机靠近纸面检测
4 个 ArUco 标记。此时相机到基座的变换是**动态**的:

    T_base_cam = T_base_ee × T_ee_cam

- T_base_ee: 当前末端位姿 (从机械臂实时读取)
- T_ee_cam:  相机在末端中的位姿 (用 calibrate_eih.py 标定)

用法:
  python3 aruco_eih.py --robot-ip 192.168.1.100 --camera-id <ID> --dry-run
  python3 aruco_eih.py --robot-ip 192.168.1.100 --camera-id <ID> --robross
"""

import argparse
import json
import os
import time
from typing import Optional

import cv2
import numpy as np

from eih_common import (
    ArucoDetector, CameraCalib, HandEyeCalib, compute_drawing_area,
    create_robot_backend, list_available_cameras, open_camera,
    pose_to_matrix, save_robross_canvas,
)


def build_dynamic_handeye(tcp_pose, T_ee_cam):
    """用当前 TCP + 标定好的 T_ee_cam 构造动态 HandEyeCalib"""
    T_base_ee = pose_to_matrix(tcp_pose)
    T_base_cam = T_base_ee @ T_ee_cam
    return HandEyeCalib(T_base_cam=T_base_cam)


def main():
    parser = argparse.ArgumentParser(
        description="臂上相机 ArUco 绘图区域检测 → RobRoss 画布标定")
    parser.add_argument("--camera-id", type=int, default=0,
                        help="摄像头 ID (默认 0)")
    parser.add_argument("--aruco-dict", default="4X4_50",
                        help="ArUco 字典 (默认 4X4_50)")
    parser.add_argument("--marker-size", type=float, default=0.02,
                        help="标记边长/米 (默认 0.02, 即 20mm)")
    parser.add_argument("--robot-ip", help="机械臂控制器 IP")
    parser.add_argument("--robot-port", type=int, default=30004)
    parser.add_argument("--camera-calib", default="camera_calib.json",
                        help="相机标定文件")
    parser.add_argument("--handeye-calib", default="eye_in_hand_calib.txt",
                        help="T_ee_cam 标定文件 (默认 eye_in_hand_calib.txt)")
    parser.add_argument("--dry-run", action="store_true",
                        help="仅检测预览，不保存")
    parser.add_argument("--robross", action="store_true",
                        help="输出 RobRoss 画布标定 YAML")
    parser.add_argument("--robross-output", default="canvas_calibration_eih.yaml",
                        help="RobRoss 画布标定输出路径")
    parser.add_argument("--start-from", default="0",
                        help="起始绘制点: 0|1|2|3 (ArUco ID) 或 center")
    parser.add_argument("--list-cameras", action="store_true",
                        help="列出所有可用摄像头设备")
    args = parser.parse_args()

    if args.list_cameras:
        list_available_cameras()
        return

    if not args.robot_ip:
        print("[✗] 臂上相机模式必须连接机械臂 (--robot-ip)，用于实时读取末端位姿")
        return

    # 标定文件默认在本目录 (eye_in_hand/) 下
    _eih_dir = os.path.dirname(os.path.abspath(__file__))
    for arg_name in ("camera_calib", "handeye_calib"):
        path = getattr(args, arg_name)
        if not os.path.exists(path):
            candidate = os.path.join(_eih_dir, os.path.basename(path))
            if os.path.exists(candidate):
                setattr(args, arg_name, candidate)

    if not os.path.exists(args.camera_calib):
        print(f"[⚠] 未找到相机内参文件: {args.camera_calib}")
    if not os.path.exists(args.handeye_calib):
        print(f"[⚠] 未找到 T_ee_cam 标定文件: {args.handeye_calib}")

    if not os.path.exists(args.camera_calib):
        print("[✗] 缺少相机标定，无法计算 3D 位姿")
        return
    camera_calib = CameraCalib.load(args.camera_calib)

    if not os.path.exists(args.handeye_calib):
        print("[✗] 缺少 T_ee_cam 标定。请先运行:")
        print("    python3 eye_in_hand/calibrate_eih.py --robot-ip <IP> --camera-id <ID>")
        return
    T_ee_cam = np.loadtxt(args.handeye_calib)
    print(f"[✓] 加载 T_ee_cam: {args.handeye_calib}")

    robot = create_robot_backend(args.robot_ip, args.robot_port)
    if robot is None:
        return

    detector = ArucoDetector(args.aruco_dict, args.marker_size, [0, 1, 2, 3])

    cap, camera_calib = open_camera(args.camera_id, camera_calib)
    if cap is None:
        return
    cmat = camera_calib.camera_matrix
    dcoeff = camera_calib.dist_coeffs

    print("=" * 55)
    if args.robross:
        print("  [Enter / Space] 保存 RobRoss 画布标定")
        print(f"  输出: {args.robross_output}")
    else:
        print("  [Enter / Space] 打印绘图区域数据")
    print("  [q]            退出")
    print("=" * 55)

    last_tcp_fetch = 0.0
    tcp_fetch_interval = 0.5
    current_tcp = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        display = frame.copy()
        h, w = display.shape[:2]

        ids, corners, poses = detector.detect(display, cmat, dcoeff)
        found = {0, 1, 2, 3}.issubset(set(ids))

        now = time.time()
        if now - last_tcp_fetch > tcp_fetch_interval:
            pose = robot.get_tcp_pose()
            if pose is not None:
                current_tcp = pose
            last_tcp_fetch = now

        # ── 状态栏 ───────────────────────────────────────────
        if found and current_tcp is not None:
            status = "✓ 4/4 已定位"
            status_color = (0, 255, 0)
        elif found:
            status = "✓ 4/4 (等待 TCP)"
            status_color = (0, 180, 255)
        else:
            status = f"✗ {len(ids)}/4"
            status_color = (0, 0, 255)
        cv2.putText(display, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

        if current_tcp is not None:
            cv2.putText(display,
                        f"TCP: ({current_tcp[0]:.3f}, {current_tcp[1]:.3f}, "
                        f"{current_tcp[2]:.3f})",
                        (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (100, 200, 255), 1)

        # ── 绘制标记 & 绘图区域 ──────────────────────────────
        area = None
        if found:
            cv2.aruco.drawDetectedMarkers(
                display, [c.reshape(1, 4, 2) for c in corners],
                np.array(ids).reshape(-1, 1))

            corners_by_id = {}
            for marker_id, c4 in zip(ids, corners):
                corners_by_id[marker_id] = c4

            order = [0, 1, 2, 3]
            all_px = np.vstack([corners_by_id[i] for i in order])
            global_px_center = all_px.mean(axis=0)

            inner_pts_px = []
            for m in order:
                c4 = corners_by_id[m]
                dists = np.linalg.norm(c4 - global_px_center, axis=1)
                inner = c4[int(np.argmin(dists))].astype(np.int32)
                inner_pts_px.append(tuple(inner))

            pts_arr = np.array(inner_pts_px, dtype=np.int32)
            cv2.polylines(display, [pts_arr], isClosed=True,
                          color=(0, 200, 0), thickness=3)

            start_id = (args.start_from if args.start_from != "center" else None)
            for i, pt in enumerate(inner_pts_px):
                marker_id = order[i]
                if start_id is not None and str(marker_id) == str(start_id):
                    cv2.circle(display, pt, 9, (0, 0, 255), -1)
                    cv2.putText(display, "  起点", (pt[0] + 10, pt[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    cv2.circle(display, pt, 4, (0, 230, 255), -1)

            cx_all = int(np.mean([p[0] for p in inner_pts_px]))
            cy_all = int(np.mean([p[1] for p in inner_pts_px]))
            cv2.circle(display, (cx_all, cy_all), 5, (0, 230, 255), -1)

            # 计算基座坐标系下的绘图区域 (动态 T_base_cam)
            if current_tcp is not None and len(poses) == 4 \
                    and all(p[1] is not None for p in poses):
                calib = build_dynamic_handeye(current_tcp, T_ee_cam)
                area = compute_drawing_area(ids, poses, calib, args.marker_size)
                if area:
                    dim = f"{area.size[0]*1000:.0f} x {area.size[1]*1000:.0f} mm"
                    cv2.putText(display, dim, (10, h - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # ── 底部提示 ─────────────────────────────────────────
        cv2.putText(display, "[Enter] 确认    [q] 退出",
                    (10, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (200, 200, 200), 1)

        cv2.imshow("ArUco (Eye-in-Hand) - AUBO Drawing Area", display)
        key = cv2.waitKey(1) & 0xFF

        is_confirm = key in (13, 10, 3, ord(" "))

        if key == ord("q"):
            break
        elif is_confirm and found:
            if current_tcp is None:
                print("\n[✗] 无法读取 TCP 位姿，无法计算基座坐标")
                continue
            if area is None:
                print("\n[✗] 无法计算 3D 坐标 — 检查相机标定文件")
                continue

            print("\n" + "═" * 55)
            print("  绘图区域数据 (JSON)")
            print("═" * 55)
            print(json.dumps(area.to_dict(), indent=2, ensure_ascii=False))
            print("─" * 55)
            print(area)
            print("─" * 55)

            if args.robross:
                save_robross_canvas(area, args.robross_output)
        elif is_confirm:
            print("[⚠] 未检测到全部 4 个标记")

    cap.release()
    cv2.destroyAllWindows()
    robot.disconnect()


if __name__ == "__main__":
    main()
