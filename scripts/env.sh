#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS2_WS="${REPO_ROOT}/ros2_ws"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi

if [[ "${WHEEL_LEG_SOURCE_WS:-1}" == "1" ]] && [[ -f "${ROS2_WS}/install/local_setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${ROS2_WS}/install/local_setup.bash"
  set -u
fi

export WHEEL_LEG_REPO_ROOT="${REPO_ROOT}"
export WHEEL_LEG_ROS2_WS="${ROS2_WS}"
export WHEEL_LEG_SCENE_FILE="${REPO_ROOT}/sim/mujoco/scenes/scence.xml"
export WHEEL_LEG_SIM_BIN="${ROS2_WS}/build/wheel_leg_simulate_ros2/wheel_leg_simulate"
