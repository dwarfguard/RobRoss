#!/usr/bin/env bash
# Run one physical-stack command behind the host-time gate and clock monitor.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PREFLIGHT="$ROOT_DIR/ros2/robross_painter/scripts/host_time_preflight.py"
MONITOR="$ROOT_DIR/ros2/robross_painter/scripts/host_clock_monitor.py"

usage() {
    printf 'Usage: %s --session-dir DIR [--session-id ID] -- COMMAND [ARG ...]\n' "$0"
}

SESSION_DIR="${HARDWARE_SESSION:-}"
SESSION_ID="${ROBROSS_SESSION_ID:-}"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --session-dir) SESSION_DIR="$2"; shift 2 ;;
        --session-id) SESSION_ID="$2"; shift 2 ;;
        --) shift; break ;;
        -h|--help) usage; exit 0 ;;
        *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
    esac
done
[[ -n "$SESSION_DIR" ]] || { printf 'A session directory is required.\n' >&2; exit 2; }
[[ $# -gt 0 ]] || { printf 'A command is required after --.\n' >&2; exit 2; }
mkdir -p "$SESSION_DIR"
SESSION_DIR="$(cd "$SESSION_DIR" && pwd)"
if [[ -z "$SESSION_ID" ]]; then
    SESSION_ID="$(basename "$SESSION_DIR")-$(cat /proc/sys/kernel/random/boot_id)"
fi

RECEIPT="$SESSION_DIR/host_time_preflight_${BASHPID}.json"
MONITOR_FILE="$SESSION_DIR/host_clock_monitor_${BASHPID}.json"
STOP_FILE="$SESSION_DIR/.stop_clock_monitor_${BASHPID}"
rm -f "$STOP_FILE"

PREFLIGHT_ARGS=(--session-id "$SESSION_ID" --receipt "$RECEIPT")
if [[ ! ( "${1:-}" == "ros2" && "${2:-}" == "launch" &&
          "${3:-}" == "aubo_ros2_driver" &&
          "${4:-}" == "aubo_control.launch.py" ) ]]; then
    PREFLIGHT_ARGS+=(--require-controller-fifo)
fi
python3 "$PREFLIGHT" "${PREFLIGHT_ARGS[@]}"
python3 "$MONITOR" --session-id "$SESSION_ID" --output "$MONITOR_FILE" \
    --stop-file "$STOP_FILE" --fail-fast &
MONITOR_PID=$!
cleanup() {
    touch "$STOP_FILE"
    wait "$MONITOR_PID" 2>/dev/null || true
    rm -f "$STOP_FILE"
}
trap cleanup EXIT INT TERM

for _ in {1..50}; do
    if python3 "$MONITOR" --validate-only --session-id "$SESSION_ID" \
        --output "$MONITOR_FILE" >/dev/null 2>&1; then
        break
    fi
    sleep 0.1
done
python3 "$MONITOR" --validate-only --session-id "$SESSION_ID" \
    --output "$MONITOR_FILE"

export ROBROSS_SESSION_ID="$SESSION_ID"
export ROBROSS_PREFLIGHT_RECEIPT="$RECEIPT"
export ROBROSS_CLOCK_MONITOR_FILE="$MONITOR_FILE"
export ROBROSS_PROVENANCE_DIR="$SESSION_DIR"
export ROBROSS_REPO="${ROBROSS_REPO:-$ROOT_DIR}"
if [[ "${1:-}" == "ros2" && "${2:-}" == "launch" && \
      "${3:-}" == "robross_painter" && \
      ( "${4:-}" == "paint.launch.py" || "${4:-}" == "teach_nudge.launch.py" ) ]]; then
    set -- "$@" \
        "qualification_run:=true" \
        "session_id:=$SESSION_ID" \
        "preflight_receipt:=$RECEIPT" \
        "clock_monitor_file:=$MONITOR_FILE"
    if [[ "${4:-}" == "paint.launch.py" ]]; then
        set -- "$@" "provenance_dir:=$SESSION_DIR"
    fi
fi

"$@" &
COMMAND_PID=$!
while kill -0 "$COMMAND_PID" 2>/dev/null; do
    if ! kill -0 "$MONITOR_PID" 2>/dev/null; then
        if wait "$MONITOR_PID"; then
            MONITOR_STATUS=0
        else
            MONITOR_STATUS=$?
        fi
        if [[ "$MONITOR_STATUS" -ne 0 ]]; then
            printf 'Host clock monitor failed; terminating physical command.\n' >&2
            kill -TERM "$COMMAND_PID" 2>/dev/null || true
            wait "$COMMAND_PID" 2>/dev/null || true
            exit 1
        fi
    fi
    sleep 0.1
done
if wait "$COMMAND_PID"; then
    COMMAND_STATUS=0
else
    COMMAND_STATUS=$?
fi
exit "$COMMAND_STATUS"
