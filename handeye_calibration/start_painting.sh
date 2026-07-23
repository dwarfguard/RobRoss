#!/usr/bin/env bash
#
# start_painting.sh — 一键启动: ArUco 检测纸张 → ROS 2 画画
#
# 用法:
#   ./start_painting.sh --camera-id 2 --paths-file /path/to/painting_paths.json
#
# 前提 (已经在运行的):
#   终端 1: ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5
#   终端 2: ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=aubo_i5
#
# 环境变量:
#   COLCON_WS — ROS 2 colcon 工作空间路径 (若 ros2 命令不在 PATH 中则需要)
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── 默认值 ──────────────────────────────────────────────────────
CAMERA_ID=""
CAMERA_CALIB="$SCRIPT_DIR/camera_calib.json"
HANDEYE_CALIB="$SCRIPT_DIR/handeye_calib.txt"
MARKER_SIZE="0.035"
ROBROSS_OUTPUT="/tmp/robross_canvas_calibration.yaml"
CALIBRATION_FILE=""    # 留空则用 paint.launch.py 的默认值 (rviz_wall_a4.yaml, 仅仿真!)
PATHS_FILE=""
ROS_WORKSPACE="${COLCON_WS:-}"

# ── 颜色 ────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

info()  { echo -e "${GREEN}[✓]${NC} $1"; }
warn()  { echo -e "${YELLOW}[⚠]${NC} $1"; }
error() { echo -e "${RED}[✗]${NC} $1"; }

# ── 解析参数 ────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --camera-id)          CAMERA_ID="$2"; shift 2 ;;
        --camera-calib)       CAMERA_CALIB="$2"; shift 2 ;;
        --handeye-calib)      HANDEYE_CALIB="$2"; shift 2 ;;
        --marker-size)        MARKER_SIZE="$2"; shift 2 ;;
        --robross-output)     ROBROSS_OUTPUT="$2"; shift 2 ;;
        --calibration-file)   CALIBRATION_FILE="$2"; shift 2 ;;
        --paths-file)         PATHS_FILE="$2"; shift 2 ;;
        --ros-workspace)      ROS_WORKSPACE="$2"; shift 2 ;;
        --help|-h)
            echo "用法: $0 --camera-id ID --paths-file PATH [选项]"
            echo ""
            echo "必选:"
            echo "  --camera-id ID       摄像头设备号 (用 --list-cameras 查看)"
            echo "  --paths-file PATH     绘画路径文件 (painting_paths.json)"
            echo ""
            echo "可选:"
            echo "  --camera-calib PATH   相机内参文件  (默认 camera_calib.json)"
            echo "  --handeye-calib PATH  手眼标定矩阵  (默认 handeye_calib.txt)"
            echo "  --marker-size M       ArUco 标记实际边长/米 (默认 0.035)"
            echo "  --robross-output PATH 画布标定 YAML 输出路径 (默认 /tmp/...)"
            echo "  --calibration-file PATH ROS 2 硬件参数 YAML"
            echo "  --ros-workspace PATH  ROS 2 colcon 工作空间路径"
            echo ""
            echo "环境变量:"
            echo "  COLCON_WS             同 --ros-workspace"
            echo ""
            echo "示例:"
            echo "  $0 --camera-id 2 \\"
            echo "    --paths-file \"\$REPO_DIR/output/demo_v1_a4_pen/painting_paths.json\" \\"
            echo "    --calibration-file \"\$COLCON_WS/src/RobRoss/ros2/robross_painter/config/hardware_a4.yaml\""
            exit 0
            ;;
        *)
            error "未知参数: $1"
            echo "    用 --help 查看用法"
            exit 1
            ;;
    esac
done

# ── 检查必选参数 ────────────────────────────────────────────────
if [[ -z "$CAMERA_ID" ]]; then
    error "必须指定 --camera-id"
    echo ""
    echo "可用摄像头:"
    python3 "$SCRIPT_DIR/aruco_drawing_area.py" --list-cameras 2>/dev/null || true
    exit 1
fi

if [[ -z "$PATHS_FILE" ]]; then
    error "必须指定 --paths-file"
    exit 1
fi

# ── 检查 ROS 2 环境 ────────────────────────────────────────────
if ! command -v ros2 &>/dev/null; then
    if [[ -n "$ROS_WORKSPACE" ]]; then
        info "加载 ROS 2 环境: $ROS_WORKSPACE/install/setup.bash"
        if [[ -f "$ROS_WORKSPACE/install/setup.bash" ]]; then
            # shellcheck source=/dev/null
            source "$ROS_WORKSPACE/install/setup.bash"
        else
            error "找不到 $ROS_WORKSPACE/install/setup.bash"
            exit 1
        fi
    else
        error "找不到 ros2 命令。请先在终端 source ROS 2 工作空间，或用 --ros-workspace 指定"
        echo "    export COLCON_WS=~/ros2_ws"
        echo "    或: $0 --ros-workspace ~/ros2_ws ..."
        exit 1
    fi
fi

# ── 文件预检查 ──────────────────────────────────────────────────
CALIB_FILE_OK=true
for f in "$CAMERA_CALIB" "$HANDEYE_CALIB" "$PATHS_FILE"; do
    if [[ ! -f "$f" ]]; then
        warn "找不到文件: $f"
        CALIB_FILE_OK=false
    fi
done
if [[ -n "$CALIBRATION_FILE" && ! -f "$CALIBRATION_FILE" ]]; then
    warn "找不到标定文件: $CALIBRATION_FILE"
fi

# ── Step 1: ArUco 检测 → 生成画布标定 YAML ────────────────────
echo ""
echo "═══════════════════════════════════════════════════"
echo "  Step 1/2: ArUco 检测纸张位置"
echo "═══════════════════════════════════════════════════"
echo "  摄像头 ID: $CAMERA_ID"
echo "  输出:      $ROBROSS_OUTPUT"
echo ""

cd "$SCRIPT_DIR"

ARUCO_ARGS=(
    --camera-id "$CAMERA_ID"
    --camera-calib "$CAMERA_CALIB"
    --handeye-calib "$HANDEYE_CALIB"
    --marker-size "$MARKER_SIZE"
    --robross
    --robross-output "$ROBROSS_OUTPUT"
)

python3 aruco_drawing_area.py "${ARUCO_ARGS[@]}"

if [[ ! -f "$ROBROSS_OUTPUT" ]]; then
    error "画布标定生成失败"
    exit 1
fi

info "画布标定已生成: $ROBROSS_OUTPUT"

# ── Step 2: ROS 2 启动画画 ─────────────────────────────────────
echo ""
echo "═══════════════════════════════════════════════════"
echo "  Step 2/2: 启动 ROS 2 画画"
echo "═══════════════════════════════════════════════════"

LAUNCH_CMD=(
    ros2 launch robross_painter paint.launch.py
    "aubo_type:=aubo_i5"
    "paths_file:=$PATHS_FILE"
    "canvas_file:=$ROBROSS_OUTPUT"
)

if [[ -n "$CALIBRATION_FILE" ]]; then
    LAUNCH_CMD+=("calibration_file:=$CALIBRATION_FILE")
fi

echo "  ${LAUNCH_CMD[*]}"
echo ""

"${LAUNCH_CMD[@]}"
