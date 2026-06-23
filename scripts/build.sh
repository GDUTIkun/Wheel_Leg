#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS2_WS="${REPO_ROOT}/ros2_ws"

if [[ $# -ne 1 ]]; then
  echo "usage: $0 {core|sim|pi|all|clean}" >&2
  exit 1
fi

PROFILE="$1"

export WHEEL_LEG_SOURCE_WS=0
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

build_ros_packages() {
  local -a packages=("$@")
  (
    cd "${ROS2_WS}"
    colcon build --packages-select "${packages[@]}" --event-handlers console_direct+
  )
}

build_mujoco_sim() {
  (
    cd "${REPO_ROOT}"
    set +u
    # shellcheck disable=SC1091
    source "${ROS2_WS}/install/local_setup.bash"
    set -u
    rm -rf "${ROS2_WS}/build/wheel_leg_simulate_ros2"
    cmake -S sim/mujoco/runtime \
      -B "${ROS2_WS}/build/wheel_leg_simulate_ros2" \
      -DWHEEL_LEG_ENABLE_ROS2=ON
    cmake --build "${ROS2_WS}/build/wheel_leg_simulate_ros2" -j"$(nproc)"
  )
}

case "${PROFILE}" in
  core)
    build_ros_packages \
      wheel_leg_msgs \
      wheel_leg_common \
      wheel_leg_bridge \
      wheel_leg_control \
      wheel_leg_hw
    ;;
  sim)
    build_ros_packages \
      wheel_leg_msgs \
      wheel_leg_common \
      wheel_leg_bridge \
      wheel_leg_control \
      wheel_leg_hw \
      wheel_leg_sim \
      wheel_leg_bringup
    build_mujoco_sim
    ;;
  pi)
    build_ros_packages \
      wheel_leg_msgs \
      wheel_leg_common \
      wheel_leg_bridge \
      wheel_leg_control \
      wheel_leg_hw \
      wheel_leg_rc \
      wheel_leg_stm32_bridge \
      wheel_leg_bringup
    ;;
  all)
    build_ros_packages \
      wheel_leg_msgs \
      wheel_leg_common \
      wheel_leg_bridge \
      wheel_leg_control \
      wheel_leg_hw \
      wheel_leg_rc \
      wheel_leg_stm32_bridge \
      wheel_leg_sim \
      wheel_leg_bringup
    build_mujoco_sim
    ;;
  clean)
    rm -rf "${ROS2_WS}/build" "${ROS2_WS}/install" "${ROS2_WS}/log"
    ;;
  *)
    echo "unknown profile: ${PROFILE}" >&2
    exit 1
    ;;
esac
