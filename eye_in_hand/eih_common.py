#!/usr/bin/env python3
"""
Eye-in-Hand 公共模块 (自包含，不依赖 handeye_calibration/)
==========================================================

本目录 (eye_in_hand/) 的所有脚本共享的代码:
  - 相机内参加载 (CameraCalib)
  - 手眼标定矩阵 (HandEyeCalib / T_ee_cam)
  - ArUco 检测器 (ArucoDetector)
  - 绘图区域计算 (compute_drawing_area / DrawingArea)
  - 机械臂通信 (JSON-RPC, JsonRpcBackend / create_robot_backend)
  - RobRoss 画布标定输出 (save_robross_canvas)
  - 摄像头枚举 (list_available_cameras)
  - 欧拉角/齐次变换工具 (rpy_to_matrix / pose_to_matrix)

本文件与 handeye_calibration/aruco_drawing_area.py 是各自独立的实现，
互不 import；如需修改检测/通信逻辑，请同步维护两处。
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import json
import socket
import os
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass, field
from abc import ABC, abstractmethod
import yaml


# ══════════════════════════════════════════════════════════════════════
#  数据结构
# ══════════════════════════════════════════════════════════════════════

@dataclass
class CameraCalib:
    """相机内参 — 从标定工具生成的 JSON 文件加载"""
    camera_matrix: np.ndarray = field(default_factory=lambda: np.eye(3))
    dist_coeffs: np.ndarray = field(default_factory=lambda: np.zeros((5, 1)))
    img_size: Tuple[int, int] = (640, 480)

    @classmethod
    def load(cls, path: str) -> "CameraCalib":
        with open(path) as f:
            data = json.load(f)
        return cls(
            camera_matrix=np.array(data["camera_matrix"]),
            dist_coeffs=np.array(data["dist_coeffs"]),
            img_size=tuple(data.get("img_size", (640, 480))),
        )


@dataclass
class HandEyeCalib:
    """
    手眼标定 — 4×4 齐次变换矩阵。
    eye-in-hand 方案中动态构造: T_base_cam = T_base_ee × T_ee_cam
    """
    T_base_cam: np.ndarray = field(default_factory=lambda: np.eye(4))

    @classmethod
    def load(cls, path: str) -> "HandEyeCalib":
        return cls(T_base_cam=np.loadtxt(path))

    def transform_point(self, p_cam: np.ndarray) -> np.ndarray:
        p = np.append(p_cam, 1.0)
        return (self.T_base_cam @ p)[:3]


@dataclass
class DrawingArea:
    """绘图区域在机械臂基座坐标系下的描述"""
    center: np.ndarray        # [x, y, z] 区域中心
    size: np.ndarray          # [width, height] (米)
    normal: np.ndarray        # [nx, ny, nz] 法向量
    corners: List[np.ndarray] # 4 角点 [tl, tr, br, bl]
    rpy: np.ndarray           # [roll, pitch, yaw] (弧度)
    marker_ids: List[int] = field(default_factory=list)

    def to_robot_pose(self) -> List[float]:
        """AUBO 目标位姿 [x, y, z, rx, ry, rz]"""
        return [round(v, 4) for v in
                [self.center[0], self.center[1], self.center[2],
                 self.rpy[0], self.rpy[1], self.rpy[2]]]

    def to_dict(self) -> dict:
        """返回可序列化的字典"""
        return {
            "center": [round(v, 4) for v in self.center],
            "size_mm": [round(self.size[0] * 1000, 1),
                        round(self.size[1] * 1000, 1)],
            "rpy_deg": [round(np.degrees(self.rpy[0]), 1),
                        round(np.degrees(self.rpy[1]), 1),
                        round(np.degrees(self.rpy[2]), 1)],
            "corners": [[round(v, 4) for v in c] for c in self.corners],
            "normal": [round(v, 4) for v in self.normal],
            "marker_ids": self.marker_ids,
            "robot_pose": self.to_robot_pose(),
        }

    def __repr__(self) -> str:
        c, s, r = self.center, self.size, np.degrees(self.rpy)
        return (
            f"  中心: [{c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f}] m\n"
            f"  尺寸: {s[0]*1000:.1f} × {s[1]*1000:.1f} mm\n"
            f"  姿态: rx={r[0]:.1f}°  ry={r[1]:.1f}°  rz={r[2]:.1f}°"
        )


# ══════════════════════════════════════════════════════════════════════
#  欧拉角 / 齐次变换工具
# ══════════════════════════════════════════════════════════════════════

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


# ══════════════════════════════════════════════════════════════════════
#  ArUco 检测器
# ══════════════════════════════════════════════════════════════════════

class ArucoDetector:
    """检测 ArUco 标记并返回 3D 位姿"""

    DICT_MAP = {
        "4X4_50": aruco.DICT_4X4_50, "4X4_100": aruco.DICT_4X4_100,
        "4X4_250": aruco.DICT_4X4_250, "4X4_1000": aruco.DICT_4X4_1000,
        "5X5_50": aruco.DICT_5X5_50, "5X5_100": aruco.DICT_5X5_100,
        "5X5_250": aruco.DICT_5X5_250, "5X5_1000": aruco.DICT_5X5_1000,
        "6X6_50": aruco.DICT_6X6_50, "6X6_100": aruco.DICT_6X6_100,
        "6X6_250": aruco.DICT_6X6_250, "6X6_1000": aruco.DICT_6X6_1000,
        "7X7_50": aruco.DICT_7X7_50, "7X7_100": aruco.DICT_7X7_100,
        "7X7_250": aruco.DICT_7X7_250, "7X7_1000": aruco.DICT_7X7_1000,
        "ORIGINAL": aruco.DICT_ARUCO_ORIGINAL,
    }

    def __init__(self, dictionary: str = "4X4_50",
                 marker_size_m: float = 0.02,
                 target_ids: Optional[List[int]] = None):
        dict_id = self.DICT_MAP.get(dictionary)
        if dict_id is None:
            raise ValueError(f"不支持字典: {dictionary}，可选: {list(self.DICT_MAP.keys())}")
        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(dict_id),
            aruco.DetectorParameters()
        )
        self.marker_size = marker_size_m
        self.target_ids = set(target_ids) if target_ids else None

    def detect(self, img: np.ndarray,
               camera_matrix: Optional[np.ndarray] = None,
               dist_coeffs: Optional[np.ndarray] = None
               ) -> Tuple[List[int], List[np.ndarray], List[Tuple[np.ndarray, np.ndarray]]]:
        corners, ids, _ = self.detector.detectMarkers(img)
        if ids is None:
            return [], [], []

        ids_flat = ids.flatten().tolist()
        corners_list = [c.reshape(4, 2) for c in corners]

        if self.target_ids:
            filtered_c, filtered_ids = [], []
            for i, mid in enumerate(ids_flat):
                if mid in self.target_ids:
                    filtered_c.append(corners_list[i])
                    filtered_ids.append(mid)
            ids_flat, corners_list = filtered_ids, filtered_c

        # 计算 3D 位姿 (solvePnP)
        poses: List[Tuple[np.ndarray, np.ndarray]] = []
        if camera_matrix is not None and corners_list:
            obj_pts = np.array([
                [-self.marker_size / 2,  self.marker_size / 2, 0],
                [ self.marker_size / 2,  self.marker_size / 2, 0],
                [ self.marker_size / 2, -self.marker_size / 2, 0],
                [-self.marker_size / 2, -self.marker_size / 2, 0],
            ], dtype=np.float32)
            for c in corners_list:
                ret, rvec, tvec = cv2.solvePnP(
                    obj_pts, c.astype(np.float32), camera_matrix, dist_coeffs)
                poses.append((rvec, tvec) if ret else (None, None))

        return ids_flat, corners_list, poses


# ══════════════════════════════════════════════════════════════════════
#  绘图区域计算
# ══════════════════════════════════════════════════════════════════════

def compute_drawing_area(ids: List[int],
                         poses: List[Tuple[np.ndarray, np.ndarray]],
                         calib: HandEyeCalib,
                         marker_size: float = 0.02) -> Optional[DrawingArea]:
    """
    从 4 个 ArUco 标记的 3D 位姿计算绘图区域。

    绘图区域由每个标记的**内侧顶点**围成 (离区域中心最近的角点):
        ID:0 ───────── ID:1
         │  ╲       ╱  │
         │    ╲   ╱    │
         │      ＋      │
         │    ╱   ╲    │
         │  ╱       ╲  │
        ID:3 ───────── ID:2
    """
    required = [0, 1, 2, 3]

    # 标记本地 4 个角点 (solvePnP 使用的 object points)
    local_corners = np.array([
        [-marker_size / 2,  marker_size / 2, 0],   # 0: 左上
        [ marker_size / 2,  marker_size / 2, 0],   # 1: 右上
        [ marker_size / 2, -marker_size / 2, 0],   # 2: 右下
        [-marker_size / 2, -marker_size / 2, 0],   # 3: 左下
    ], dtype=np.float64)

    marker_corners_3d: Dict[int, np.ndarray] = {}
    marker_centers: Dict[int, np.ndarray] = {}

    for mid, (rvec, tvec) in zip(ids, poses):
        if mid in required and tvec is not None and rvec is not None:
            R, _ = cv2.Rodrigues(rvec)
            pts_3d = (R @ local_corners.T).T + tvec.flatten()
            marker_corners_3d[mid] = pts_3d
            marker_centers[mid] = tvec.flatten()

    if len(marker_centers) < 4:
        missing = [m for m in required if m not in marker_centers]
        print(f"[✗] 缺少标记: {missing}")
        return None

    # 全局中心 (所有 16 个角点平均)
    all_pts = np.vstack([marker_corners_3d[m] for m in required])
    global_center = np.mean(all_pts, axis=0)

    # 找每个标记的内侧顶点 (离全局中心最近)
    inner_corners_cam: Dict[int, np.ndarray] = {}
    for m in required:
        pts = marker_corners_3d[m]
        dists = np.linalg.norm(pts - global_center, axis=1)
        inner_idx = int(np.argmin(dists))
        inner_corners_cam[m] = pts[inner_idx].copy()

    # 变换到机械臂基座坐标系
    inner_base = {m: calib.transform_point(inner_corners_cam[m]) for m in required}

    # 计算区域参数
    all_c = np.array([inner_base[m] for m in required])
    center = np.mean(all_c, axis=0)

    w = (np.linalg.norm(inner_base[1] - inner_base[0]) +
         np.linalg.norm(inner_base[2] - inner_base[3])) / 2.0
    h = (np.linalg.norm(inner_base[3] - inner_base[0]) +
         np.linalg.norm(inner_base[2] - inner_base[1])) / 2.0

    vx = inner_base[1] - inner_base[0]
    vy = inner_base[3] - inner_base[0]
    normal = np.cross(vx, vy)
    nrm = np.linalg.norm(normal)
    if nrm > 0:
        normal /= nrm
    if normal[2] < 0:
        normal = -normal

    # 旋转矩阵 → RPY
    x_axis = vx / np.linalg.norm(vx)
    y_axis = vy / np.linalg.norm(vy)
    z_axis = normal
    x_axis = x_axis - np.dot(x_axis, z_axis) * z_axis
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    R_mat = np.column_stack([x_axis, y_axis, z_axis])

    sy = np.sqrt(R_mat[0, 0]**2 + R_mat[1, 0]**2)
    if sy > 1e-6:
        roll = np.arctan2(R_mat[2, 1], R_mat[2, 2])
        pitch = np.arctan2(-R_mat[2, 0], sy)
        yaw = np.arctan2(R_mat[1, 0], R_mat[0, 0])
    else:
        roll = np.arctan2(-R_mat[1, 2], R_mat[1, 1])
        pitch = np.arctan2(-R_mat[2, 0], sy)
        yaw = 0.0

    return DrawingArea(
        center=center, size=np.array([w, h]), normal=normal,
        corners=[inner_base[m] for m in required],
        rpy=np.array([roll, pitch, yaw]), marker_ids=required,
    )


# ══════════════════════════════════════════════════════════════════════
#  机械臂通信 — JSON-RPC (纯 socket，无 pyaubo_sdk 依赖)
# ══════════════════════════════════════════════════════════════════════

class RobotBackend(ABC):
    """机械臂通信抽象接口"""

    @abstractmethod
    def connect(self, ip: str, port: int) -> bool: ...
    @abstractmethod
    def disconnect(self): ...
    @abstractmethod
    def login(self) -> bool: ...
    @abstractmethod
    def move_line(self, pose: List[float], a: float, v: float,
                  blend: float, duration: float) -> bool: ...
    @abstractmethod
    def move_joint(self, q: List[float], a: float, v: float,
                   blend: float, duration: float) -> bool: ...
    @abstractmethod
    def get_tcp_pose(self) -> Optional[List[float]]: ...
    @abstractmethod
    def set_speed(self, fraction: float): ...


class JsonRpcBackend(RobotBackend):
    """JSON-RPC 后端 (纯 socket，兼容所有平台)"""

    def __init__(self, robot_name: str = "rob1"):
        self.robot_name = robot_name
        self.sock: Optional[socket.socket] = None
        self._rpc_id = 0

    def connect(self, ip: str, port: int = 30004) -> bool:
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((ip, port))
            print(f"[✓] JSON-RPC 已连接 {ip}:{port}")
            return True
        except Exception as e:
            print(f"[✗] JSON-RPC 连接失败: {e}")
            return False

    def disconnect(self):
        if self.sock:
            self.sock.close()
            self.sock = None

    def _call(self, method: str, params: List[Any] = None,
              quiet: bool = False) -> Optional[Any]:
        if not self.sock:
            raise ConnectionError("未连接")
        self._rpc_id += 1
        req = json.dumps({"jsonrpc": "2.0", "method": method,
                          "params": params or [], "id": self._rpc_id}) + "\n"
        try:
            self.sock.sendall(req.encode())
            resp = b""
            while True:
                chunk = self.sock.recv(4096)
                if not chunk:
                    break
                resp += chunk
                try:
                    data = json.loads(resp.decode())
                    break
                except json.JSONDecodeError:
                    continue
            if "error" in data:
                if not quiet:
                    print(f"[✗] RPC 错误: {data['error']}")
                return None
            return data.get("result")
        except Exception as e:
            if not quiet:
                print(f"[✗] RPC 失败 [{method}]: {e}")
            return None

    def login(self) -> bool:
        # 不同控制器版本的登录方法名不同，甚至无需登录。
        # 逐个尝试，全部不存在 (method not found) 时视为无需登录。
        for method in ("robot_interface.login", "robot.login", "login"):
            if self._call(method, quiet=True) is not None:
                return True
        return True

    def move_line(self, pose, a=0.3, v=0.2, blend=0.0, duration=0.0) -> bool:
        return self._call(f"{self.robot_name}.MotionControl.moveLine",
                          [pose, a, v, blend, duration]) is not None

    def move_joint(self, q, a=0.5, v=0.5, blend=0.0, duration=0.0) -> bool:
        return self._call(f"{self.robot_name}.MotionControl.moveJoint",
                          [q, a, v, blend, duration]) is not None

    def get_tcp_pose(self):
        return self._call(f"{self.robot_name}.RobotState.getTcpPose")

    def set_speed(self, fraction: float):
        self._call(f"{self.robot_name}.MotionControl.setSpeedFraction", [fraction])


def create_robot_backend(ip: str, port: int = 30004,
                         robot_name: str = "rob1") -> Optional[RobotBackend]:
    """创建 JSON-RPC 后端（通过 TCP 直连机械臂控制器）"""
    backend = JsonRpcBackend(robot_name)
    print(f"[i] 使用 JSON-RPC 后端 ({ip}:{port})")
    if not backend.connect(ip, port):
        return None
    if not backend.login():
        print("[✗] 登录失败")
        backend.disconnect()
        return None
    return backend


# ══════════════════════════════════════════════════════════════════════
#  RobRoss 画布标定输出
# ══════════════════════════════════════════════════════════════════════

def rotmat_to_quat(R: np.ndarray) -> tuple:
    """旋转矩阵 → 四元数 (x, y, z, w)"""
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        return ((R[2, 1] - R[1, 2]) / s,
                (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s,
                0.25 * s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return (0.25 * s,
                (R[0, 1] + R[1, 0]) / s,
                (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s)
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        return ((R[0, 1] + R[1, 0]) / s,
                0.25 * s,
                (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s)
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        return ((R[0, 2] + R[2, 0]) / s,
                (R[1, 2] + R[2, 1]) / s,
                0.25 * s,
                (R[1, 0] - R[0, 1]) / s)


def save_robross_canvas(area: DrawingArea, output_path: str):
    """
    把 ArUco 检测到的绘图区域保存为 RobRoss 画布标定文件。

    坐标映射:
      - ArUco ID:0 内侧顶点 → canvas_origin_xyz (画布左上角)
      - ArUco ID:0→ID:1 方向 → canvas 的 x 轴 (从左到右)
      - ArUco ID:0→ID:3 方向 → canvas 的 y 轴 (从上到下)

    RobRoss 中的绘画路径使用此文件作为 canvas_file 参数。
    """
    # 画布左上角 (ID:0 内侧顶点)
    idx0 = area.marker_ids.index(0)
    top_left = area.corners[idx0]  # 单位: 米 (base frame)

    # canvas x 轴 = ID:0 → ID:1
    idx1 = area.marker_ids.index(1)
    canvas_x = area.corners[idx1] - area.corners[idx0]
    canvas_x /= np.linalg.norm(canvas_x)

    # canvas y 轴 = ID:0 → ID:3 (注意: paper y 朝下)
    idx3 = area.marker_ids.index(3)
    canvas_y = area.corners[idx3] - area.corners[idx0]
    canvas_y /= np.linalg.norm(canvas_y)

    # canvas z = x × y (垂直纸面向外)
    canvas_z = np.cross(canvas_x, canvas_y)
    canvas_z /= np.linalg.norm(canvas_z)

    # 确保正交
    canvas_y = np.cross(canvas_z, canvas_x)

    R_canvas = np.column_stack([canvas_x, canvas_y, canvas_z])
    qx, qy, qz, qw = rotmat_to_quat(R_canvas)

    # 画布尺寸 (mm)
    width_mm = round(area.size[0] * 1000, 1)
    height_mm = round(area.size[1] * 1000, 1)

    header = (
        "# Canvas calibration from ArUco markers (eye-in-hand).\n"
        f"# Drawing area: {width_mm} x {height_mm} mm\n"
        f"# top_left      ID:0 = {top_left.tolist()}\n"
        f"# Pass to paint.launch.py as: canvas_file:=<this file>\n"
    )
    data = {
        "painting_executor": {
            "ros__parameters": {
                "canvas_origin_xyz": [round(float(v), 6) for v in top_left],
                "canvas_quat_xyzw": [
                    round(float(qx), 6), round(float(qy), 6),
                    round(float(qz), 6), round(float(qw), 6),
                ],
            }
        }
    }
    with open(output_path, "w") as f:
        f.write(header)
        yaml.dump(data, f, default_flow_style=None, sort_keys=False)

    print(f"\n[✓] RobRoss 画布标定已保存: {output_path}")
    print(f"    canvas_origin_xyz: {[round(float(v), 6) for v in top_left]}")
    print(f"    canvas_quat_xyzw:  {[round(float(v), 6) for v in (qx, qy, qz, qw)]}")
    print(f"    画布尺寸: {width_mm} x {height_mm} mm")
    print(f"\n  RobRoss 中使用:")
    print(f"    ros2 launch robross_painter paint.launch.py \\")
    print(f"      aubo_type:=aubo_i5 \\")
    print(f"      calibration_file:=<your_calibration.yaml> \\")
    print(f"      paths_file:=<your_paths.json> \\")
    print(f"      canvas_file:={output_path}")
    print(f"\n  calibration_file 示例: ros2/robross_painter/config/hardware_a4.yaml")


# ══════════════════════════════════════════════════════════════════════
#  摄像头枚举
# ══════════════════════════════════════════════════════════════════════

def list_available_cameras(max_id: int = 10):
    """枚举系统上所有可用摄像头设备"""
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_fd = os.dup(2)
    os.dup2(devnull, 2)
    os.close(devnull)
    try:
        print("=" * 55)
        print("  扫描可用摄像头...")
        print("=" * 55)
        found = []
        for i in range(max_id):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = cap.get(cv2.CAP_PROP_FPS)
                ret, _ = cap.read()
                status = "✓" if ret else "?"
                info = f"  │  [{status}]  ID:{i}   {w}×{h}  {fps:.0f} fps"
                if cap.getBackendName():
                    info += f"  [{cap.getBackendName()}]"
                print(info)
                found.append(i)
                cap.release()
            else:
                cap.release()
        if not found:
            print("  │  (未找到可用摄像头)")
        print("=" * 55)
        if found:
            print(f"  可用 ID: {found}")
            print(f"  使用: python3 aruco_eih.py --camera-id <ID>")
        return found
    finally:
        os.dup2(old_fd, 2)
        os.close(old_fd)
