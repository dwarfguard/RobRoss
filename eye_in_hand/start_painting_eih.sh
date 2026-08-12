#!/usr/bin/env bash
# Deprecated name retained for discoverability. This helper localizes the
# canvas only; it never launches ROS or commands robot motion.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROBOT_IP=""
CAMERA_ID=""
CAMERA_CALIB="$SCRIPT_DIR/camera_calib.json"
HANDEYE_CALIB="$SCRIPT_DIR/eye_in_hand_calib.txt"
MARKER_SIZE="0.02"
OUTPUT_FILE="/tmp/robross_canvas_calibration_eih.yaml"

usage() {
    cat <<EOF
Usage: $0 --robot-ip IP --camera-id ID [options]

Generate eye-in-hand canvas-localization YAML and stop. This script does not
launch painting. Continue with docs/hardware_run_guide.md after reviewing the
reported canvas dimensions.

Required:
  --robot-ip IP          Robot controller IP used to read the active TCP pose
  --camera-id ID         Camera device ID

Options:
  --camera-calib PATH    Camera intrinsics (default: eye_in_hand/camera_calib.json)
  --handeye-calib PATH   Camera/TCP transform (default: eye_in_hand/eye_in_hand_calib.txt)
  --marker-size METERS   Corner-marker black-square size (default: 0.02)
  --output PATH          Output YAML (default: /tmp/robross_canvas_calibration_eih.yaml)
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --robot-ip) ROBOT_IP="$2"; shift 2 ;;
        --camera-id) CAMERA_ID="$2"; shift 2 ;;
        --camera-calib) CAMERA_CALIB="$2"; shift 2 ;;
        --handeye-calib) HANDEYE_CALIB="$2"; shift 2 ;;
        --marker-size) MARKER_SIZE="$2"; shift 2 ;;
        --output|--robross-output) OUTPUT_FILE="$2"; shift 2 ;;
        --help|-h) usage; exit 0 ;;
        *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ -z "$ROBOT_IP" || -z "$CAMERA_ID" ]]; then
    usage >&2
    exit 2
fi

for path in "$CAMERA_CALIB" "$HANDEYE_CALIB"; do
    if [[ ! -f "$path" ]]; then
        printf 'Required file not found: %s\n' "$path" >&2
        exit 1
    fi
done

python3 "$SCRIPT_DIR/aruco_eih.py" \
    --robot-ip "$ROBOT_IP" \
    --camera-id "$CAMERA_ID" \
    --camera-calib "$CAMERA_CALIB" \
    --handeye-calib "$HANDEYE_CALIB" \
    --marker-size "$MARKER_SIZE" \
    --robross \
    --robross-output "$OUTPUT_FILE"

if [[ ! -f "$OUTPUT_FILE" ]]; then
    printf 'Canvas localization was not written: %s\n' "$OUTPUT_FILE" >&2
    exit 1
fi

printf '\nCanvas localization written: %s\n' "$OUTPUT_FILE"
printf 'No painting process was launched. Review the reported dimensions and continue with:\n'
printf '  docs/hardware_run_guide.md\n'
