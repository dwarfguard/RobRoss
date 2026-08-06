#!/usr/bin/env python3
"""
相机标定快速工具
===============
1. 打印棋盘格: python3 calibrate_camera.py --board
2. 标定: python3 calibrate_camera.py
3. 结果: camera_calib.json

操作: 拿棋盘格在摄像头前缓慢旋转/倾斜，按 Space 拍照
      至少拍 10~15 张不同角度，按 Enter 完成标定
"""

import cv2
import numpy as np
import argparse
import os
import json


def generate_board(path="chessboard.png", cols=9, rows=6, square_px=80):
    """生成棋盘格图片，打印后贴在硬板上"""
    board = np.ones((rows * square_px, cols * square_px), dtype=np.uint8) * 255
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                y1, x1 = r * square_px, c * square_px
                board[y1:y1 + square_px, x1:x1 + square_px] = 0
    cv2.imwrite(path, board)
    print(f"[✓] 棋盘格已保存: {path}")
    print(f"    内角点: {cols}×{rows}")
    print(f"    打印后测量一个格子的边长(mm) → 就是你用的 square_size")


def calibrate(cam_id=0, cols=9, rows=6, square_mm=25.0, out="camera_calib.json"):
    """实时标定"""
    square_size = square_mm / 1000.0  # mm → m

    cap = cv2.VideoCapture(cam_id)
    if not cap.isOpened():
        print(f"[✗] 无法打开摄像头 ID={cam_id}")
        return

    obj_points = []   # 3D
    img_points = []   # 2D

    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size

    print("\n" + "=" * 55)
    print("  相机标定")
    print("  把棋盘格在摄像头前缓慢旋转/倾斜")
    print("  [Space] 拍照  [Enter] 完成标定  [q] 退出")
    print("=" * 55)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        display = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)

        if found:
            corners_sub = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                           (cv2.TERM_CRITERIA_EPS +
                                            cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            cv2.drawChessboardCorners(display, (cols, rows), corners_sub, found)

        info = f"已采集: {len(obj_points)} 张"
        cv2.putText(display, info, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0) if found else (0, 0, 255), 2)
        cv2.putText(display, "[Space]拍照 [Enter]标定 [q]退出",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.imshow("Camera Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        elif key in (32, 13, 10, 3):  # Space or Enter
            if key == 32 and found:    # Space: 采集
                obj_points.append(objp)
                img_points.append(corners_sub)
                print(f"   ✓ 采集第 {len(obj_points)} 张")
            elif key in (13, 10, 3):   # Enter: 标定
                if len(obj_points) < 5:
                    print(f"   ⚠ 至少 5 张, 当前 {len(obj_points)}")
                    continue
                h, w = frame.shape[:2]
                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                    obj_points, img_points, (w, h), None, None)

                data = {
                    "camera_matrix": mtx.tolist(),
                    "dist_coeffs": dist.tolist(),
                    "img_size": [w, h],
                    "rms_error_px": round(ret, 4),
                }
                with open(out, "w") as f:
                    json.dump(data, f, indent=2)
                print(f"\n[✓] 标定完成! 重投影误差: {ret:.3f} px")
                print(f"[✓] 已保存: {out}")
                print(f"\n  fx={mtx[0,0]:.1f}  fy={mtx[1,1]:.1f}")
                print(f"  cx={mtx[0,2]:.1f}  cy={mtx[1,2]:.1f}")
                print(f"\n  错误 < 0.5 px 为优秀, < 1.0 px 为可用")
                print(f"  过大则需要重新采集更多角度")
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="相机标定工具")
    parser.add_argument("--board", action="store_true", help="生成棋盘格图片")
    parser.add_argument("--cam-id", type=int, default=0, help="摄像头 ID")
    parser.add_argument("--cols", type=int, default=9, help="棋盘格内角点列数")
    parser.add_argument("--rows", type=int, default=6, help="棋盘格内角点行数")
    parser.add_argument("--square-mm", type=float, default=25.0,
                        help="格子边长(mm), 打印后用尺子量!")
    parser.add_argument("--out", default="camera_calib.json", help="输出文件")
    args = parser.parse_args()

    if args.board:
        generate_board()
    else:
        calibrate(args.cam_id, args.cols, args.rows, args.square_mm, args.out)
