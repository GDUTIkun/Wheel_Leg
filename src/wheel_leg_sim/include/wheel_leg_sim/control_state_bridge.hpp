#ifndef WHEEL_LEG_SIM__CONTROL_STATE_BRIDGE_HPP_
#define WHEEL_LEG_SIM__CONTROL_STATE_BRIDGE_HPP_

#include <wheel_leg_control/stand_control_types.hpp>

#include "wheel_leg_sim/sensor_types.hpp"

namespace wheel_leg_sim {

wheel_leg_control::StandControlState BuildStandControlState(
    const RobotSensorData& sensor_data);

}  // namespace wheel_leg_sim

#endif  // WHEEL_LEG_SIM__CONTROL_STATE_BRIDGE_HPP_
