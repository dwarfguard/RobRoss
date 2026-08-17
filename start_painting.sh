#!/usr/bin/env bash
# Current physical painting entry point: arm-mounted camera workflow.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$ROOT_DIR/eye_in_hand/start_painting_eih.sh" "$@"
