#ifndef WHEEL_LEG_SIM__ROBOT_STATE_ASSEMBLY_HPP_
#define WHEEL_LEG_SIM__ROBOT_STATE_ASSEMBLY_HPP_

#include <array>

#include "wheel_leg_sim/sensor_types.hpp"

namespace wheel_leg_sim {

struct RawLegInputs {
  JointState hip;
  JointState knee;
  JointState wheel;
  JointState calf;
};

RobotSensorData AssembleRobotSensorData(
    const std::array<double, 4>& base_quat,
    const std::array<double, 3>& base_gyro,
    const std::array<double, 3>& base_accel,
    double world_velocity_x,
    double world_velocity_y,
    double dt,
    const RawLegInputs& left_leg_inputs,
    const RawLegInputs& right_leg_inputs,
    SensorAssemblyState* state);

}  // namespace wheel_leg_sim

#endif  // WHEEL_LEG_SIM__ROBOT_STATE_ASSEMBLY_HPP_
