#include "wheel_leg_sim/control_state_bridge.hpp"

namespace wheel_leg_sim {

wheel_leg_control::StandControlState BuildStandControlState(
    const RobotSensorData& sensor_data) {
  wheel_leg_control::StandControlState control_state;
  control_state.body.distance = sensor_data.base_link.distance;
  control_state.body.velocity = sensor_data.base_link.velocity;
  control_state.body.pitch = sensor_data.base_link.pitch;
  control_state.body.pitch_rate = sensor_data.base_link.pitch_rate;
  control_state.body.yaw_rate = sensor_data.base_link.yaw_rate;

  control_state.left_leg.hip_absolute =
      sensor_data.left_leg.kinematics.hip_absolute;
  control_state.left_leg.calf_absolute =
      sensor_data.left_leg.kinematics.calf_absolute;
  control_state.left_leg.leg_length =
      sensor_data.left_leg.kinematics.leg_length;
  control_state.left_leg.phi = sensor_data.left_leg.kinematics.phi;
  control_state.left_leg.phi_rate =
      sensor_data.left_leg.kinematics.phi_rate;

  control_state.right_leg.hip_absolute =
      sensor_data.right_leg.kinematics.hip_absolute;
  control_state.right_leg.calf_absolute =
      sensor_data.right_leg.kinematics.calf_absolute;
  control_state.right_leg.leg_length =
      sensor_data.right_leg.kinematics.leg_length;
  control_state.right_leg.phi = sensor_data.right_leg.kinematics.phi;
  control_state.right_leg.phi_rate =
      sensor_data.right_leg.kinematics.phi_rate;
  return control_state;
}

}  // namespace wheel_leg_sim
