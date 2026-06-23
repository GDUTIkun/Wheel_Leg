#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $# -ne 1 ]]; then
  echo "usage: $0 {rc|hw|hw_core}" >&2
  exit 1
fi

MODE="$1"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

case "${MODE}" in
  rc)
    ros2 launch wheel_leg_bringup rc.launch.py
    ;;
  hw)
    ros2 launch wheel_leg_bringup hw.launch.py
    ;;
  hw_core)
    ros2 launch wheel_leg_bringup hw.launch.py use_stm32_bridge:=false
    ;;
  *)
    echo "unknown mode: ${MODE}" >&2
    exit 1
    ;;
esac
