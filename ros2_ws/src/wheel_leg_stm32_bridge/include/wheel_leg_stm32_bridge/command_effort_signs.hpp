#ifndef WHEEL_LEG_STM32_BRIDGE__COMMAND_EFFORT_SIGNS_HPP_
#define WHEEL_LEG_STM32_BRIDGE__COMMAND_EFFORT_SIGNS_HPP_

#include <array>

namespace wheel_leg_stm32_bridge {

// Maps ROS joint torque semantics to the motor-side polarity validated on
// hardware during the single-joint probe test sequence.
inline constexpr std::array<float, 6> kCommandEffortSigns = {
    -1.0f,
    1.0f,
    -1.0f,
    1.0f,
    1.0f,
    -1.0f,
};

}  // namespace wheel_leg_stm32_bridge

#endif  // WHEEL_LEG_STM32_BRIDGE__COMMAND_EFFORT_SIGNS_HPP_
