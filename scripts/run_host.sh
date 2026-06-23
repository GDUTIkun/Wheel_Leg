#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $# -ne 1 ]]; then
  echo "usage: $0 {sim|sim_only|control_only}" >&2
  exit 1
fi

MODE="$1"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

case "${MODE}" in
  sim)
    ros2 launch wheel_leg_bringup sim.launch.py
    ;;
  sim_only)
    ros2 launch wheel_leg_bringup sim.launch.py run_controller:=false
    ;;
  control_only)
    ros2 run wheel_leg_control wheel_leg_controller_node
    ;;
  *)
    echo "unknown mode: ${MODE}" >&2
    exit 1
    ;;
esac
