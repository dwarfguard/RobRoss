#!/usr/bin/env python3
"""
臂上相机手眼标定 (Eye-in-Hand)
================================

目的: 求解 T_ee_cam — 相机固定在机械臂末端时，相机坐标系在末端坐标系中的位姿。

方法: AX = XB
  A = 机械臂末端位姿 T_base_ee (从机械臂控制器读取)
  B = 固定标记在相机坐标系中的位姿 T_cam_marker (从 ArUco solvePnP 求解)
  X = T_ee_cam (待求解)

操作流程:
  1. 把 ArUco 标记固定在桌面上 (标定过程中不能移动!)
  2. 机械臂切换到拖拽模式，相机装在末端
  3. 把机械臂拖到不同位姿 (角度、远近都要变化)，使标记出现在画面中
  4. 每个位姿按 [Space] 记录 — 自动读机械臂 TCP + 摄像头检测标记
  5. 至少 3 个位姿，建议 6~8 个
  6. 按 [c] 完成 → 得到 eye_in_hand_calib.txt

用法:
  python3 calibrate_eih.py --robot-ip 192.168.1.100 --camera-id <ID>
  python3 calibrate_eih.py --robot-ip 192.168.1.100 --dry-run
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "handeye_calibration"))
from aruco_drawing_area import (
    ArucoDetector, CameraCalib, create_robot_backend
)


def rpy_to_matrix(rx, ry, rz):
    """RPY 欧拉角 (弧度) → 3×3 旋转矩阵 (ZYX 顺序, 与 getTcpPose 约定一致)"""
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx


def pose_to_matrix(pose):
    """[x,y,z,rx,ry,rz] → 4×4 齐次变换矩阵"""
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(pose[3], pose[4], pose[5])
    T[:3, 3] = pose[:3]
    return T


def rvec_tvec_to_matrix(rvec, tvec):
    """solvePnP 的 rvec/tvec → 4×4 齐次变换矩阵"""
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T


def rotation_vector_of(R):
    """旋转矩阵 → 旋转向量 (矩阵对数)"""
    rvec, _ = cv2.Rodrigues(R)
    return rvec.flatten()


def solve_ax_xb(A_list, B_list):
    """
    求解 AX = XB (Park-Martin 方法)

    A_i = T_base_ee_i, B_i = T_cam_marker_i, X = T_ee_cam
    对所有 i≠j: A_i⁻¹ A_j = X (B_i B_j⁻¹) X⁻¹
    → 旋转: alpha_ij = log(A_i⁻¹ A_j), beta_ij = log(B_i B_j⁻¹)
      alpha_ij = R_X * beta_ij  (正交 Procrustes 求解)
    → 平移: 最小二乘
    """
    n = len(A_list)
    pairs = [(i, j) for i in range(n) for j in range(i + 1, n)]

    A_rot = []  # alpha 向量列表
    B_rot = []
    for i, j in pairs:
        Aij = np.linalg.inv(A_list[i]) @ A_list[j]
        Bij = B_list[i] @ np.linalg.inv(B_list[j])
        A_rot.append(rotation_vector_of(Aij[:3, :3]))
        B_rot.append(rotation_vector_of(Bij[:3, :3]))

    # 旋转: min Σ ||alpha - R_X beta||²  → Procrustes
    A_mat = np.vstack(A_rot)   # (m,3)
    B_mat = np.vstack(B_rot)   # (m,3)
    M = A_mat.T @ B_mat         # 3×3
    U, _, Vt = np.linalg.svd(M)
    R_X = U @ Vt
    if np.linalg.det(R_X) < 0:
        Vt[-1] *= -1
        R_X = U @ Vt

    # 平移: 展开 A_ij X = X B_ij 得 (R_A - I) t_X = R_X t_B - t_A
    rows_A, rows_b = [], []
    for i, j in pairs:
        Aij = np.linalg.inv(A_list[i]) @ A_list[j]
        Bij = B_list[i] @ np.linalg.inv(B_list[j])
        R_A, t_A = Aij[:3, :3], Aij[:3, 3]
        R_B, t_B = Bij[:3, :3], Bij[:3, 3]
        rows_A.append(R_A - np.eye(3))
        rows_b.append(R_X @ t_B - t_A)
    coeff = np.vstack(rows_A)
    target = np.hstack(rows_b)
    t_X, *_ = np.linalg.lstsq(coeff, target, rcond=None)

    X = np.eye(4)
    X[:3, :3] = R_X
    X[:3, 3] = t_X
    return X


def load_camera_calib(path="camera_calib.json"):
    if not os.path.exists(path):
        print(f"[⚠] 未找到 {path}")
        return None, None
    calib = CameraCalib.load(path)
    print(f"[✓] 加载相机内参: fx={calib.camera_matrix[0,0]:.1f} fy={calib.camera_matrix[1,1]:.1f}")
    return calib.camera_matrix, calib.dist_coeffs


def main():
    parser = argparse.ArgumentParser(description="臂上相机手眼标定 (Eye-in-Hand, AX=XB)")
    parser.add_argument("--camera-id", type=int, default=0, help="摄像头 ID")
    parser.add_argument("--camera-calib", default="camera_calib.json",
                        help="相机标定文件 (在 handeye_calibration/ 下)")
    parser.add_argument("--aruco-dict", default="4X4_50", help="ArUco 字典")
    parser.add_argument("--marker-size", type=float, default=0.02,
                        help="标记边长/米")
    parser.add_argument("--output", default="eye_in_hand_calib.txt",
                        help="输出文件 (默认 eye_in_hand_calib.txt)")
    parser.add_argument("--robot-ip", help="机械臂 IP (自动读取 TCP 位姿)")
    parser.add_argument("--robot-port", type=int, default=30004)
    parser.add_argument("--dry-run", action="store_true", help="看流程")
    args = parser.parse_args()

    if args.dry_run:
        print("\n" + "=" * 55)
        print("  臂上相机手眼标定 (Eye-in-Hand)")
        print("=" * 55)
        print()
        print("  ★ 不需要笔尖碰标记!")
        print("    标记固定不动，机械臂只需摆出不同位姿让相机看清标记")
        print("    建议距离 10~25cm (D405 最佳 7~30cm，太近会失焦)")
        print()
        print("  1. 把 ArUco 标记固定在桌面上，标定过程中不能移动!")
        print("  2. 相机固定在机械臂末端，机械臂切到拖拽模式")
        print("  3. 把机械臂拖到不同位姿，使标记出现在画面中")
        print("  4. 每个位姿按 [Space] 记录")
        print("     → 自动读机械臂 TCP 位姿 + 摄像头检测标记")
        print("  5. 至少 3 个位姿，建议 6~8 个 (角度、远近都要变)")
        print("  6. 按 [c] 完成 → 得到 eye_in_hand_calib.txt")
        print()
        print("  运行: python3 calibrate_eih.py --robot-ip 192.168.1.100 --camera-id <ID>")
        print("=" * 55)
        return

    if not args.robot_ip:
        print("[✗] 需要 --robot-ip <IP> (自动读取 TCP 位姿)")
        return

    # 相机标定文件默认在 handeye_calibration/ 下
    if not os.path.exists(args.camera_calib):
        fallback = os.path.join(os.path.dirname(__file__), "..",
                                "handeye_calibration", args.camera_calib)
        if os.path.exists(fallback):
            args.camera_calib = fallback
    cmat, dist = load_camera_calib(args.camera_calib)
    if cmat is None:
        return

    robot = create_robot_backend(args.robot_ip, args.robot_port)
    if robot is None:
        return

    detector = ArucoDetector(args.aruco_dict, args.marker_size, target_ids=None)

    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        print(f"[✗] 无法打开摄像头 ID={args.camera_id}")
        return

    A_list = []  # T_base_ee
    B_list = []  # T_cam_marker

    # 缓存最后一次检测到的标记位姿 (机械臂可能挡住标记)
    cached_rvec = None
    cached_tvec = None
    cached_mid = None

    # 实时 TCP 刷新 (限频)
    last_tcp_fetch = 0.0
    tcp_fetch_interval = 0.5
    current_pose = None

    print("\n" + "=" * 55)
    print("  臂上相机手眼标定 (Eye-in-Hand)")
    print("=" * 55)
    print()
    print("  ★ 不需要笔尖碰标记!")
    print("    标记固定不动，机械臂摆出不同位姿让相机看清标记即可")
    print("    建议距离 10~25cm")
    print()
    print("  操作:")
    print("    [Space]  采集 — 记录当前位姿 + 标记位姿")
    print("    [c]      标定完成并保存")
    print("    [q]      退出")
    print()

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        display = frame.copy()
        h, w = display.shape[:2]

        ids, corners, poses = detector.detect(display, cmat, dist)
        marker_found = len(ids) > 0

        now = time.time()
        if now - last_tcp_fetch > tcp_fetch_interval:
            pose = robot.get_tcp_pose()
            if pose is not None:
                current_pose = pose
            last_tcp_fetch = now

        # 当前标记检测
        current_rvec = current_tvec = current_mid = None
        if marker_found and len(poses) > 0 and poses[0][1] is not None:
            current_mid = ids[0]
            current_rvec = poses[0][0]
            current_tvec = poses[0][1].flatten()
            cached_mid = current_mid
            cached_rvec = current_rvec
            cached_tvec = current_tvec
            cv2.drawFrameAxes(display, cmat, dist,
                              current_rvec, current_tvec,
                              args.marker_size * 0.6)

        # ── 信息叠加 (左上角简洁显示) ─────────────────────────
        n_history = len(A_list)
        if current_mid is not None:
            status = f"✓  ID:{current_mid}"
            status_color = (0, 220, 80)
            cache_note = ""
        elif cached_mid is not None:
            status = f"◉  缓存 ID:{cached_mid}"
            status_color = (0, 180, 255)
            cache_note = "  (标记被遮挡，使用缓存)"
        else:
            status = "✗  未检测到"
            status_color = (0, 0, 230)
            cache_note = ""

        line1 = f"已采集: {n_history} 组  |  检测: {status}{cache_note}"
        cv2.putText(display, line1, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 2)

        if current_tvec is not None:
            cv2.putText(display,
                        f"Marker: ({current_tvec[0]:.3f}, {current_tvec[1]:.3f}, "
                        f"{current_tvec[2]:.3f})  m",
                        (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 220, 80), 1)

        if current_pose is not None:
            cv2.putText(display,
                        f"TCP:    ({current_pose[0]:.3f}, {current_pose[1]:.3f}, "
                        f"{current_pose[2]:.3f})  m",
                        (10, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (100, 200, 255), 1)
            cv2.putText(display, "●", (330, 74),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1)

        # 已采集位姿列表 (右上角)
        if n_history > 0:
            list_x = w - 320
            list_h = min(n_history, 5) * 18 + 28
            cv2.rectangle(display, (list_x, 8), (list_x + 310, 8 + list_h),
                          (20, 20, 20), -1)
            list_roi = display[8:8 + list_h, list_x:list_x + 310]
            cv2.addWeighted(list_roi, 0.7, np.full_like(list_roi, 20), 0.3, 0, list_roi)
            cv2.putText(display, f"─ 位姿: {n_history} 组 ─",
                        (list_x + 8, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
            for i in range(min(n_history, 5)):
                a = A_list[i]
                cv2.putText(display,
                            f"#{i+1:02d}  xyz({a[0,3]:.3f}, {a[1,3]:.3f}, {a[2,3]:.3f})",
                            (list_x + 8, 48 + i * 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (160, 200, 230), 1)

        cv2.putText(display, "[Space] 采集    [c] 完成    [q] 退出",
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (200, 200, 200), 1)

        cv2.imshow("Eye-in-Hand Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        elif key == 32:
            # 优先用当前检测，标记被遮挡时用缓存
            if current_rvec is not None and current_tvec is not None:
                rvec, tvec = current_rvec, current_tvec
                mid = current_mid
            elif cached_rvec is not None and cached_tvec is not None:
                rvec, tvec = cached_rvec, cached_tvec
                mid = cached_mid
                print("\n  [i] 标记被遮挡，使用缓存的标记位姿")
            else:
                print("\n  [✗] 未检测到标记，也无法获取缓存")
                continue

            if current_pose is None:
                pose = robot.get_tcp_pose()
                if pose is None:
                    print("\n  [✗] 读取 TCP 失败，再试一次")
                    continue
                current_pose = pose

            A = pose_to_matrix(current_pose)
            B = rvec_tvec_to_matrix(rvec, tvec)
            A_list.append(A)
            B_list.append(B)
            print(f"\n  ID:{mid}  TCP: ({current_pose[0]:.4f}, {current_pose[1]:.4f}, "
                  f"{current_pose[2]:.4f})")
            print(f"          Marker: ({tvec[0]:.4f}, {tvec[1]:.4f}, {tvec[2]:.4f})")
            print(f"  [✓] 第 {len(A_list)} 组")

        elif key == ord("c"):
            if len(A_list) < 3:
                print(f"\n  [✗] 至少 3 个位姿，当前 {len(A_list)}")
                continue

            X = solve_ax_xb(A_list, B_list)

            # 验证: T_base_marker = A_i @ X @ B_i 应恒定
            T_marks = [A_list[i] @ X @ B_list[i] for i in range(len(A_list))]
            center = np.mean(T_marks, axis=0)
            pos_errors = [np.linalg.norm(Tm[:3, 3] - center[:3, 3]) * 1000
                          for Tm in T_marks]
            rot_errors = []
            for Tm in T_marks:
                dR = center[:3, :3].T @ Tm[:3, :3]
                rot_errors.append(np.degrees(np.linalg.norm(rotation_vector_of(dR))))

            print(f"\n  ┌─ 结果: T_ee_cam (相机在末端坐标系中的位姿)")
            print(f"  │  R:")
            for row in X[:3, :3]:
                print(f"  │    [{row[0]:.6f}, {row[1]:.6f}, {row[2]:.6f}]")
            print(f"  │  t: [{X[0,3]:.4f}, {X[1,3]:.4f}, {X[2,3]:.4f}]")
            print(f"  │  位姿一致性: 平均位置误差 {np.mean(pos_errors):.2f} mm, "
                  f"平均角度误差 {np.mean(rot_errors):.2f}°")
            print(f"  │  最大位置误差: {np.max(pos_errors):.2f} mm")
            print(f"  └─")

            if np.max(pos_errors) > 15 or np.max(rot_errors) > 5:
                inp = input("  误差较大，仍然保存? (y/n): ")
                if inp.lower() != "y":
                    continue

            np.savetxt(args.output, X, fmt="%.6f")
            print(f"  [✓] 已保存: {args.output}")
            print(f"      aruco_eih.py 会自动加载")
            break

    cap.release()
    cv2.destroyAllWindows()
    if robot:
        robot.disconnect()


if __name__ == "__main__":
    main()
