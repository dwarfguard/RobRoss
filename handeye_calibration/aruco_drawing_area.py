#!/usr/bin/env python3
"""
ArUco 绘制区域检测与 AUBO 机械臂定位
=======================================

相机检测 4 个 ArUco → 计算绘图区域 → 发送给机械臂

通信后端自动选择:
  1. pyaubo_sdk (官方 SDK，Linux/Windows)
  2. JSON-RPC over TCP (纯 socket，macOS 开发调试用)

ArUco 布局（从相机视角看）:
    +-----------------------+
    | [] ID:0       ID:1 [] |
    |                       |
    |      绘图区域          |
    |                       |
    | [] ID:3       ID:2 [] |
    +-----------------------+

绘图区域由 4 个标记的**内侧顶点**围成:
    ID:0 ───────── ID:1
     │  ╲       ╱  │
     │    ╲   ╱    │
     │      ＋      │   ← 区域中心
     │    ╱   ╲    │
     │  ╱       ╲  │
    ID:3 ───────── ID:2

用法:
  python3 aruco_drawing_area.py --robot-ip 192.168.1.100
  python3 aruco_drawing_area.py --list-cameras
  python3 aruco_drawing_area.py --camera-id 2 --dry-run
  python3 aruco_drawing_area.py --image photo.jpg
  python3 aruco_drawing_area.py --generate-markers
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import json
import socket
import time
import argparse
import os
import sys
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass, field
from abc import ABC, abstractmethod
import yaml

# ── 尝试导入 pyaubo_sdk ──────────────────────────────────────────
try:
    from pyaubo_sdk import AuboApi as _AuboApi
    SDK_AVAILABLE = True
except ModuleNotFoundError:
    SDK_AVAILABLE = False


# ══════════════════════════════════════════════════════════════════════
#  数据结构
# ══════════════════════════════════════════════════════════════════════

@dataclass
class CameraCalib:
    """相机内参 — 从你的标定工具生成的 JSON 文件加载"""
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
    手眼标定 — 4×4 齐次变换矩阵 T_base_cam
    将相机坐标系下的点变换到机械臂基座坐标系。
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
                 marker_size_m: float = 0.035,
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
                         marker_size: float = 0.035) -> Optional[DrawingArea]:
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
#  机械臂通信 — 统一接口 + 双后端
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


class SdkBackend(RobotBackend):
    """官方 pyaubo_sdk 后端"""

    def __init__(self, robot_name: str = "rob1"):
        self.robot_name = robot_name

    def connect(self, ip: str, port: int = 8899) -> bool:
        try:
            self._api = _AuboApi()
            self._api.getRobotNames()
            self._robot_iface = self._api.getRobotInterface()
            self._motion = self._robot_iface.getMotionControl()
            self._state = self._robot_iface.getRobotState()
            print(f"[✓] SDK 后端已创建 (robot={self.robot_name})")
            return True
        except Exception as e:
            print(f"[✗] SDK 后端初始化失败: {e}")
            return False

    def disconnect(self):
        self._api = None

    def login(self) -> bool:
        return True

    def move_line(self, pose, a=0.3, v=0.2, blend=0.0, duration=0.0) -> bool:
        return self._motion.moveLine(pose, a, v, blend, duration) == 0

    def move_joint(self, q, a=0.5, v=0.5, blend=0.0, duration=0.0) -> bool:
        return self._motion.moveJoint(q, a, v, blend, duration) == 0

    def get_tcp_pose(self):
        return self._state.getTcpPose()

    def set_speed(self, fraction: float):
        self._motion.setSpeedFraction(fraction)


class JsonRpcBackend(RobotBackend):
    """JSON-RPC 后端 (纯 socket，兼容所有平台)"""

    def __init__(self, robot_name: str = "rob1"):
        self.robot_name = robot_name
        self.sock: Optional[socket.socket] = None
        self._rpc_id = 0

    def connect(self, ip: str, port: int = 8899) -> bool:
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

    def _call(self, method: str, params: List[Any] = None) -> Optional[Any]:
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
                print(f"[✗] RPC 错误: {data['error']}")
                return None
            return data.get("result")
        except Exception as e:
            print(f"[✗] RPC 失败 [{method}]: {e}")
            return None

    def login(self) -> bool:
        return self._call("robot_interface.login") is not None

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


def create_robot_backend(ip: str, port: int = 8899,
                         robot_name: str = "rob1") -> Optional[RobotBackend]:
    """自动选择通信后端: 优先 pyaubo_sdk，否则 JSON-RPC"""
    backend = SdkBackend(robot_name) if SDK_AVAILABLE else JsonRpcBackend(robot_name)
    label = "pyaubo_sdk" if SDK_AVAILABLE else "JSON-RPC"
    print(f"[i] 使用 {label} 后端")
    if not SDK_AVAILABLE:
        print("    (macOS 调试用; Linux 机械臂上安装 pyaubo_sdk 后自动切换)")
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
        "# Canvas calibration from ArUco markers.\n"
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
            print(f"  使用: python3 aruco_drawing_area.py --camera-id <ID>")
        return found
    finally:
        os.dup2(old_fd, 2)
        os.close(old_fd)


# ══════════════════════════════════════════════════════════════════════
#  主流程
# ══════════════════════════════════════════════════════════════════════

def send_to_robot(robot: RobotBackend, area: DrawingArea, start_from: str = "0"):
    """
    发送绘图区域到机械臂。

    Args:
        robot:      机械臂后端
        area:       绘图区域
        start_from: 起始点—ArUco 标记 ID (0~3) 或 "center"
    """
    center_pose = area.to_robot_pose()
    rpy = area.rpy  # [roll, pitch, yaw]

    # 确定目标点在 base 坐标系下的 [x, y, z]
    if start_from == "center":
        target_point = area.center.copy()
        target_label = "区域中心"
    else:
        target_id = int(start_from)
        if target_id not in area.marker_ids:
            print(f"[✗] start_from={start_from} 不在已检测的标记中 ({area.marker_ids})")
            return
        idx = area.marker_ids.index(target_id)
        target_point = area.corners[idx].copy()
        target_label = f"ID:{target_id} 内侧顶点"

    # 构建目标位姿 (位置 = 目标点, 姿态 = 区域姿态)
    target_pose = [
        round(target_point[0], 4),
        round(target_point[1], 4),
        round(target_point[2], 4),
        round(rpy[0], 4),
        round(rpy[1], 4),
        round(rpy[2], 4),
    ]

    print("\n" + "┌─ 绘图区域信息")
    print(f"│  中心: {center_pose}")
    print(f"│  尺寸: {area.size[0]*1000:.1f} × {area.size[1]*1000:.1f} mm")
    print(f"│  姿态: rx={np.degrees(rpy[0]):.1f}°  "
          f"ry={np.degrees(rpy[1]):.1f}°  "
          f"rz={np.degrees(rpy[2]):.1f}°")
    print("│  边界角点 (基座坐标系):")
    for i, (mid, c) in enumerate(zip(area.marker_ids, area.corners)):
        marker = " ← 起点" if str(mid) == start_from else ""
        print(f"│    P{i}(ID:{mid}) = ({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f}){marker}")

    print(f"├─ 移动到 {target_label} 上方...")
    approach = target_pose.copy()
    approach[2] += 0.05
    if robot.move_line(approach, a=0.3, v=0.2):
        print(f"│  ✓ 到达安全高度")
        time.sleep(0.3)
        print(f"├─ 下降到 {target_label}...")
        if robot.move_line(target_pose, a=0.2, v=0.1):
            print(f"│  ✓ 到达 {target_label}")
        else:
            print("│  ✗ 下降失败")
    else:
        print("│  ✗ 运动失败")
    print("└─")


def main():
    parser = argparse.ArgumentParser(
        description="ArUco 绘图区域检测 → AUBO 机械臂定位")
    parser.add_argument("--camera-id", type=int, default=0,
                        help="摄像头 ID (默认 0)")
    parser.add_argument("--aruco-dict", default="4X4_50",
                        help="ArUco 字典 (默认 4X4_50)")
    parser.add_argument("--marker-size", type=float, default=0.035,
                        help="标记边长/米 (默认 0.035)")
    parser.add_argument("--robot-ip",
                        help="机械臂控制器 IP")
    parser.add_argument("--robot-port", type=int, default=8899,
                        help="JSON-RPC 端口 (默认 8899)")
    parser.add_argument("--camera-calib", default="camera_calib.json",
                        help="相机标定文件 (默认 camera_calib.json)")
    parser.add_argument("--handeye-calib", default="handeye_calib.txt",
                        help="手眼标定文件 (默认 handeye_calib.txt)")
    parser.add_argument("--dry-run", action="store_true",
                        help="仅检测预览，不连接机械臂")
    parser.add_argument("--image",
                        help="处理单张图片 (不打开摄像头)")
    parser.add_argument("--list-cameras", action="store_true",
                        help="列出所有可用摄像头设备")
    parser.add_argument("--generate-markers", action="store_true",
                        help="生成 ArUco 标记图片")
    parser.add_argument("--test-comm", action="store_true",
                        help="测试与机械臂的通信")
    parser.add_argument("--start-from", default="0",
                        help="起始绘制点: 0|1|2|3 (ArUco ID) 或 center (默认 0)")
    parser.add_argument("--robross", action="store_true",
                        help="输出 RobRoss 画布标定 YAML 而非直接控制机械臂")
    parser.add_argument("--robross-output", default="canvas_calibration.yaml",
                        help="RobRoss 画布标定输出路径 (默认 canvas_calibration.yaml)")

    args = parser.parse_args()

    # ── 列出摄像头 ──────────────────────────────────────────────
    if args.list_cameras:
        list_available_cameras()
        return

    # ── 生成标记 ────────────────────────────────────────────────
    if args.generate_markers:
        dict_id = ArucoDetector.DICT_MAP.get(args.aruco_dict)
        if dict_id is None:
            print(f"[✗] 不支持的字典: {args.aruco_dict}")
            return
        adict = aruco.getPredefinedDictionary(dict_id)
        os.makedirs("markers", exist_ok=True)
        for mid in [0, 1, 2, 3]:
            img = np.zeros((200, 200), dtype=np.uint8)
            img = aruco.generateImageMarker(adict, mid, 200, img, 1)
            cv2.imwrite(f"markers/aruco_{args.aruco_dict}_ID{mid}.png", img)
            print(f"[✓] markers/aruco_{args.aruco_dict}_ID{mid}.png")
        print("\n打印标记，测量实际边长，传给 --marker-size 参数")
        return

    # ── 通信测试 ────────────────────────────────────────────────
    if args.test_comm:
        ip = args.robot_ip
        if not ip:
            print("[i] Dry-run 模式 (不连接)")
            return
        robot = create_robot_backend(ip, args.robot_port)
        if robot:
            pose = robot.get_tcp_pose()
            print(f"[✓] 当前 TCP: {pose}" if pose else "[✗] 获取位姿失败")
            robot.disconnect()
        return

    # ── 加载标定 ────────────────────────────────────────────────
    camera_calib = None
    if os.path.exists(args.camera_calib):
        camera_calib = CameraCalib.load(args.camera_calib)
        print(f"[✓] 加载相机标定: {args.camera_calib}")
    else:
        print(f"[⚠] 未找到 {args.camera_calib}，使用默认内参")

    handeye = None
    if os.path.exists(args.handeye_calib):
        handeye = HandEyeCalib.load(args.handeye_calib)
        print(f"[✓] 加载手眼标定: {args.handeye_calib}")
    else:
        handeye = HandEyeCalib()
        print(f"[⚠] 未找到 {args.handeye_calib}，使用单位矩阵")

    # ── 连接机械臂 ──────────────────────────────────────────────
    robot: Optional[RobotBackend] = None
    if args.robot_ip and not args.dry_run and not args.robross:
        robot = create_robot_backend(args.robot_ip, args.robot_port)

    # ── 检测器 ──────────────────────────────────────────────────
    detector = ArucoDetector(args.aruco_dict, args.marker_size, [0, 1, 2, 3])
    cmat = camera_calib.camera_matrix if camera_calib else None
    dcoeff = camera_calib.dist_coeffs if camera_calib else None

    # ── 单张图片模式 ───────────────────────────────────────────
    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print(f"[✗] 无法读取 {args.image}")
            return
        ids, corners, poses = detector.detect(img, cmat, dcoeff)
        if len(ids) == 4:
            if not poses or not poses[0][1]:
                print("\n[✗] 无法计算 3D 坐标 — 需要相机标定文件 camera_calib.json")
            else:
                area = compute_drawing_area(ids, poses, handeye, args.marker_size)
                if area:
                    print(f"\n{area}")
                    if args.robross:
                        save_robross_canvas(area, args.robross_output)
                    elif robot:
                        send_to_robot(robot, area, args.start_from)
        cv2.imshow("Result", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        if robot:
            robot.disconnect()
        return

    # ── 摄像头实时模式 ──────────────────────────────────────────
    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        print(f"[✗] 无法打开摄像头 ID={args.camera_id}")
        if robot:
            robot.disconnect()
        return
    print(f"[✓] OpenCV 摄像头已打开 (ID={args.camera_id})")

    print("=" * 55)
    if args.robross:
        print("  [Enter / Space] 保存 RobRoss 画布标定")
        print(f"  输出: {args.robross_output}")
    else:
        print("  [Enter / Space] 发送坐标到机械臂")
    print("  [q]            退出")
    print("=" * 55)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        display = frame.copy()

        ids, corners, poses = detector.detect(display, cmat, dcoeff)
        found = {0, 1, 2, 3}.issubset(set(ids))

        # ── 绘制 ArUco 标记 & 绘图区域边界 ────────────────────
        if found:
            cv2.aruco.drawDetectedMarkers(
                display, [c.reshape(1, 4, 2) for c in corners],
                np.array(ids).reshape(-1, 1))

            # 按 ID 建立映射
            corners_by_id: Dict[int, np.ndarray] = {}
            for marker_id, c4 in zip(ids, corners):
                corners_by_id[marker_id] = c4

            # 找内侧顶点 (离全局像素中心最近的角点)
            order = [0, 1, 2, 3]
            all_px = np.vstack([corners_by_id[i] for i in order])
            global_px_center = all_px.mean(axis=0)

            inner_pts_px = []
            for m in order:
                c4 = corners_by_id[m]
                dists = np.linalg.norm(c4 - global_px_center, axis=1)
                inner = c4[int(np.argmin(dists))].astype(np.int32)
                inner_pts_px.append(tuple(inner))

            # 绘制绿色四边形 (内侧顶点连线)
            pts_arr = np.array(inner_pts_px, dtype=np.int32)
            cv2.polylines(display, [pts_arr], isClosed=True,
                          color=(0, 200, 0), thickness=3)

            # 绘制内侧顶点 + 高亮起点
            start_id = (args.start_from if args.start_from != "center" else None)
            for i, pt in enumerate(inner_pts_px):
                marker_id = order[i]
                if start_id is not None and str(marker_id) == str(start_id):
                    cv2.circle(display, pt, 9, (0, 0, 255), -1)  # 红色大圆
                    cv2.putText(display, "  起点", (pt[0] + 10, pt[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    cv2.circle(display, pt, 4, (0, 230, 255), -1)  # 黄色小圆

            # 区域中心 + 标签
            cx_all = int(np.mean([p[0] for p in inner_pts_px]))
            cy_all = int(np.mean([p[1] for p in inner_pts_px]))
            center_color = (0, 0, 255) if start_id == "center" else (0, 230, 255)
            cv2.circle(display, (cx_all, cy_all), 5, center_color, -1)
            label = "  绘图区域 (起点=中心)" if start_id == "center" else "  绘图区域"
            cv2.putText(display, label, (cx_all + 8, cy_all + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 230, 255), 2)

            # 叠加物理尺寸 (3D 位姿可用时)
            if len(poses) == 4 and all(p[1] is not None for p in poses):
                area = compute_drawing_area(ids, poses, handeye, args.marker_size)
                if area:
                    dim = f"{area.size[0]*1000:.0f} x {area.size[1]*1000:.0f} mm"
                    cv2.putText(display, dim, (10, display.shape[0] - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # ── 坐标轴: 在绘图区域中心画 X(红) Y(绿) Z(蓝) ──
                # 计算平均位姿 (4 个标记的均值) 作为区域中心在相机坐标系下的位姿
                cm = cmat
                dc = dcoeff
                if cm is None:
                    h, w = display.shape[:2]
                    cm = np.array([[w, 0, w/2], [0, w, h/2], [0, 0, 1]], dtype=np.float32)
                    dc = np.zeros((5, 1), dtype=np.float32)

                # 平均平移
                avg_t = np.mean([poses[i][1].flatten() for i in range(4)], axis=0)
                # 平均旋转 (旋转矩阵均值 → SVD 再正交化)
                Rs = []
                for i in range(4):
                    R, _ = cv2.Rodrigues(poses[i][0])
                    Rs.append(R)
                avg_R = np.mean(Rs, axis=0)
                U, _, Vt = np.linalg.svd(avg_R)
                avg_R = U @ Vt
                # 翻转 Z 轴 → 指向纸内
                avg_R[:, 2] = -avg_R[:, 2]
                avg_rvec, _ = cv2.Rodrigues(avg_R)

                axis_len = max(area.size[0], area.size[1]) * 0.3  # 轴长 = 区域尺寸 30%
                cv2.drawFrameAxes(display, cm, dc, avg_rvec, avg_t, axis_len)

                # 轴标签 (投影到图像)
                center_2d, _ = cv2.projectPoints(
                    np.array([[0, 0, 0]], dtype=np.float32),
                    avg_rvec, avg_t, cm, dc)
                ox, oy = map(int, center_2d[0][0])
                for label, pt_w, color in [
                    ("X", (axis_len, 0, 0), (0, 0, 255)),
                    ("Y", (0, axis_len, 0), (0, 255, 0)),
                    ("Z", (0, 0, -axis_len), (255, 0, 0)),
                ]:
                    end_2d, _ = cv2.projectPoints(
                        np.array([pt_w], dtype=np.float32),
                        avg_rvec, avg_t, cm, dc)
                    px, py = map(int, end_2d[0][0])
                    if 0 <= px < display.shape[1] and 0 <= py < display.shape[0]:
                        cv2.arrowedLine(display, (ox, oy), (px, py), color, 2, tipLength=0.15)
                        cv2.putText(display, label, (px + 5, py - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # ── 状态栏 ───────────────────────────────────────────
        status = "✓ 4/4 已定位" if found else f"✗ {len(ids)}/4"
        cv2.putText(display, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0) if found else (0, 0, 255), 2)

        cv2.imshow("ArUco - AUBO Drawing Area", display)
        key = cv2.waitKey(1) & 0xFF

        # Enter 在不同系统上的键码可能不同:
        #   13 = '\r' (Windows/Linux highgui, macOS 部分后端)
        #   10 = '\n' (macOS 部分后端)
        #   3  = Enter (macOS Cocoa 后端)
        # 空格键 (32) 作为备用确认键
        is_confirm = key in (13, 10, 3, ord(" "))

        if key == ord("q"):
            break
        elif is_confirm and found:
            # 检查 3D 位姿是否可计算
            poses_ok = all(p[1] is not None and p[0] is not None for p in poses)
            if not poses_ok:
                print("\n[✗] 无法计算 3D 坐标 — 缺少相机标定文件")
                print("   请将你的标定数据保存为 camera_calib.json，格式:")
                print('   {"camera_matrix": [[fx,0,cx],[0,fy,cy],[0,0,1]],')
                print('    "dist_coeffs": [[k1,k2,p1,p2,k3]]}')
                print("   或指定路径: --camera-calib /path/to/calib.json")
            else:
                area = compute_drawing_area(ids, poses, handeye, args.marker_size)
                if area:
                    data = area.to_dict()
                    print("\n" + "═" * 55)
                    print("  绘图区域数据 (JSON)")
                    print("═" * 55)
                    print(json.dumps(data, indent=2, ensure_ascii=False))
                    print("─" * 55)
                    print(area)
                    print("─" * 55)
                    if args.robross:
                        save_robross_canvas(area, args.robross_output)
                    elif robot:
                        send_to_robot(robot, area, args.start_from)
        elif is_confirm:
            print("[⚠] 未检测到全部 4 个标记")

    cap.release()
    cv2.destroyAllWindows()
    if robot:
        robot.disconnect()


if __name__ == "__main__":
    main()
