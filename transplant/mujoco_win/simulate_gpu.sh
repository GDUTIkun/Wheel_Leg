#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCENE_FILE="${1:-$SCRIPT_DIR/scence.xml}"
SIM_BIN="${MUJOCO_SIMULATE_BIN:-/opt/mujoco-3.7.0/bin/simulate}"

exec env \
  __NV_PRIME_RENDER_OFFLOAD=1 \
  __GLX_VENDOR_LIBRARY_NAME=nvidia \
  "$SIM_BIN" "$SCENE_FILE"
