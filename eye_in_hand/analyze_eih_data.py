#!/usr/bin/env python3
"""
离线分析 eye-in-hand 标定采集数据 — 诊断标定质量
==================================================

用法:
  python3 analyze_eih_data.py eye_in_hand_calib_data.npz

分析内容:
  1. 旋转轴分布 — AX=XB 是否退化 (第一奇异值占比 > 0.85 即退化)
  2. 位姿覆盖 — 距离范围、角度变化范围
  3. 一致性误差 — 用当前数据解出的 X 下, 标记在基座中是否恒定
"""

import sys

import cv2
import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from calibrate_eih import check_pose_diversity, solve_ax_xb, rotation_vector_of


def main():
    if len(sys.argv) < 2:
        print("用法: python3 analyze_eih_data.py <data.npz>")
        return

    data = np.load(sys.argv[1])
    A_list = [data["A_list"][i] for i in range(len(data["A_list"]))]
    B_list = [data["B_list"][i] for i in range(len(data["B_list"]))]
    n = len(A_list)
    print(f"位姿数量: {n}")

    # ── 1. 旋转轴多样性 ──────────────────────────────────────
    n_pairs, s_norm = check_pose_diversity(A_list, B_list)
    print(f"\n[1] 旋转轴分布 (奇异值占比): x={s_norm[0]:.2f} y={s_norm[1]:.2f} z={s_norm[2]:.2f}")
    if s_norm[0] > 0.85:
        print("    ⚠ 退化! 旋转轴几乎都在同一方向 — 旋转解不可靠")
        print("    解决: 采集时让相机倾斜/旋转/俯仰, 而不是只平移")
    else:
        print("    ✓ 旋转轴分布合理")

    # ── 2. 位姿覆盖范围 ──────────────────────────────────────
    dists = [np.linalg.norm(B_list[i][:3, 3]) for i in range(n)]
    print(f"\n[2] 标记距离范围: {min(dists)*1000:.0f} ~ {max(dists)*1000:.0f} mm")
    if max(dists) - min(dists) < 0.05:
        print("    ⚠ 距离变化很小 — 平移分量约束弱")

    # 旋转角范围 (相对第一个位姿)
    angles = []
    for i in range(1, n):
        dR = A_list[0][:3, :3].T @ A_list[i][:3, :3]
        rvec, _ = cv2.Rodrigues(dR)
        angles.append(np.degrees(np.linalg.norm(rvec)))
    if angles:
        print(f"    末端相对姿态变化: {min(angles):.1f}° ~ {max(angles):.1f}°")

    # ── 3. 一致性误差 ────────────────────────────────────────
    if n >= 3:
        X = solve_ax_xb(A_list, B_list)
        T_marks = [A_list[i] @ X @ B_list[i] for i in range(n)]
        center = np.mean(T_marks, axis=0)
        pos_errors = [np.linalg.norm(Tm[:3, 3] - center[:3, 3]) * 1000
                      for Tm in T_marks]
        rot_errors = []
        for Tm in T_marks:
            dR = center[:3, :3].T @ Tm[:3, :3]
            rot_errors.append(np.degrees(np.linalg.norm(rotation_vector_of(dR))))
        print(f"\n[3] 一致性误差 (标记在基座中应恒定):")
        print(f"    位置: 平均 {np.mean(pos_errors):.2f} mm, 最大 {np.max(pos_errors):.2f} mm")
        print(f"    旋转: 平均 {np.mean(rot_errors):.2f}°, 最大 {np.max(rot_errors):.2f}°")
        if np.mean(rot_errors) > 5:
            print("    ⚠ 旋转误差大 — 大概率是旋转轴分布退化或 solvePnP 噪声")
        elif np.mean(pos_errors) > 15:
            print("    ⚠ 位置误差大 — 检查 marker-size 是否正确、TCP 读数是否稳定")


if __name__ == "__main__":
    main()
