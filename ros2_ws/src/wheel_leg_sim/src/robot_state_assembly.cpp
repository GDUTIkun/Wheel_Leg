#include "wheel_leg_sim/robot_state_assembly.hpp"

#include "wheel_leg_sim/attitude_utils.hpp"
#include "wheel_leg_sim/base_state_assembly.hpp"
#include "wheel_leg_sim/leg_kinematics.hpp"
#include "wheel_leg_sim/leg_state_assembly.hpp"

namespace wheel_leg_sim {

RobotSensorData AssembleRobotSensorData(
    const std::array<double, 4>& base_quat,
    const std::array<double, 3>& base_gyro,
    const std::array<double, 3>& base_accel,
    double world_velocity_x,
    double world_velocity_y,
    double dt,
    const RawLegInputs& left_leg_inputs,
    const RawLegInputs& right_leg_inputs,
    SensorAssemblyState* state) {
  RobotSensorData sensor_data;
  if (!state) {
    return sensor_data;
  }

  const EulerAngles base_euler = QuaternionToEuler(base_quat);
  const double base_forward_velocity =
      ComputeBaseForwardVelocity(world_velocity_x, world_velocity_y,
                                 base_euler.yaw);
  sensor_data.base_link = AssembleBaseLinkState(
      base_euler,
      base_gyro[0],
      base_gyro[1],
      base_gyro[2],
      base_accel[0],
      base_accel[1],
      base_accel[2],
      base_forward_velocity,
      state,
      dt);

  sensor_data.left_leg = AssembleLeftLegState(
      left_leg_inputs.hip,
      left_leg_inputs.knee,
      left_leg_inputs.wheel,
      left_leg_inputs.calf);
  sensor_data.right_leg = AssembleRightLegState(
      right_leg_inputs.hip,
      right_leg_inputs.knee,
      right_leg_inputs.wheel,
      right_leg_inputs.calf);

  UpdatePhiRate(&sensor_data.left_leg.kinematics, &state->previous_left_phi,
                &state->filtered_left_phi_rate, &state->has_previous_left_phi,
                dt);
  UpdatePhiRate(&sensor_data.right_leg.kinematics, &state->previous_right_phi,
                &state->filtered_right_phi_rate, &state->has_previous_right_phi,
                dt);
  return sensor_data;
}

}  // namespace wheel_leg_sim
