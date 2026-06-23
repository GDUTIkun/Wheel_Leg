#ifndef WHEEL_LEG_HW__INTERFACE_CONTRACT_HPP_
#define WHEEL_LEG_HW__INTERFACE_CONTRACT_HPP_

#include <array>
#include <string_view>

namespace wheel_leg_hw {

inline constexpr std::array<std::string_view, 6> kCanonicalJointNames = {
    "left_hip",
    "left_knee",
    "left_wheel",
    "right_hip",
    "right_knee",
    "right_wheel",
};

inline constexpr std::string_view kImuFrame = "base_link";

}  // namespace wheel_leg_hw

#endif  // WHEEL_LEG_HW__INTERFACE_CONTRACT_HPP_
