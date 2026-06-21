#ifndef WHEEL_LEG_CONTROL__STATE_MESSAGE_CONVERSIONS_HPP_
#define WHEEL_LEG_CONTROL__STATE_MESSAGE_CONVERSIONS_HPP_

#include <wheel_leg_common/types.hpp>
#include <wheel_leg_msgs/msg/stand_control_state.hpp>

#include "wheel_leg_control/stand_control_types.hpp"

namespace wheel_leg_control {

StandControlState FromRosStandControlState(
    const wheel_leg_msgs::msg::StandControlState& msg);

wheel_leg_msgs::msg::StandControlState ToRosStandControlState(
    const StandControlState& state,
    const wheel_leg_common::TimePoint& stamp);

}  // namespace wheel_leg_control

#endif  // WHEEL_LEG_CONTROL__STATE_MESSAGE_CONVERSIONS_HPP_
