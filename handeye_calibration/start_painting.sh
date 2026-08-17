#!/usr/bin/env bash
# The fixed-camera workflow was retired; preserve the old entry point fail-closed.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
printf '%s\n' \
    'The fixed-camera handeye_calibration workflow has been retired.' \
    'Use the current arm-mounted workflow; its physical host gate is mandatory.' >&2
exec "$ROOT_DIR/eye_in_hand/start_painting_eih.sh" "$@"
