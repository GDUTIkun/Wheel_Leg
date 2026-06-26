#include "wheel_leg_stm32_bridge/hardware_state_assembler.hpp"

#include <cmath>

namespace wheel_leg_stm32_bridge {

double NormalizeAngleDelta(double angle_delta) {
  while (angle_delta > kHardwareStatePi) {
    angle_delta -= 2.0 * kHardwareStatePi;
  }
  while (angle_delta < -kHardwareStatePi) {
    angle_delta += 2.0 * kHardwareStatePi;
  }
  return angle_delta;
}

HardwareLegKinematics ComputeHardwareLegKinematics(
    double hip_joint_position,
    double knee_joint_position,
    double hip_offset_rad,
    double knee_offset_rad,
    double previous_phi,
    double filtered_phi_rate,
    bool has_previous_phi,
    double dt,
    const HardwareStateAssemblerConfig& config) {
  const double hip_absolute = hip_joint_position + hip_offset_rad;
  const double calf_absolute =
      kHardwareStatePi - hip_offset_rad + knee_joint_position + knee_offset_rad;
  const double theta_l2 = hip_absolute + calf_absolute - kHardwareStatePi;
  const double x = config.thigh_length * std::cos(hip_absolute) +
                   config.calf_length * std::cos(theta_l2);
  const double y_clockwise = config.thigh_length * std::sin(hip_absolute) +
                             config.calf_length * std::sin(theta_l2);

  HardwareLegKinematics output;
  output.hip_absolute = hip_absolute;
  output.calf_absolute = calf_absolute;
  output.leg_length = std::sqrt(x * x + y_clockwise * y_clockwise);
  output.phi = std::atan2(y_clockwise, x);

  if (dt <= 0.0 || !has_previous_phi) {
    output.phi_rate = 0.0;
  } else {
    const double raw_phi_rate =
        NormalizeAngleDelta(output.phi - previous_phi) / dt;
    output.phi_rate = config.phi_rate_low_pass_alpha * filtered_phi_rate +
                      (1.0 - config.phi_rate_low_pass_alpha) * raw_phi_rate;
  }

  return output;
}

HardwareStateAssemblyOutput AssembleHardwareState(
    const HardwareStateAssemblyInput& input,
    double dt,
    HardwareStateAssemblyState* state,
    const HardwareStateAssemblerConfig& config) {
  HardwareStateAssemblyOutput output;
  if (state == nullptr) {
    return output;
  }

  output.body_velocity =
      0.5 * (input.joint_velocity[2] + input.joint_velocity[5]) *
      config.wheel_radius;
  if (dt > 0.0) {
    state->body_distance += output.body_velocity * dt;
  }
  output.body_distance = state->body_distance;

  output.left_leg = ComputeHardwareLegKinematics(
      input.joint_position[0], input.joint_position[1],
      config.left_hip_offset_rad, config.left_knee_offset_rad,
      state->previous_left_phi, state->filtered_left_phi_rate,
      state->has_previous_left_phi, dt, config);
  state->previous_left_phi = output.left_leg.phi;
  state->filtered_left_phi_rate = output.left_leg.phi_rate;
  state->has_previous_left_phi = true;

  output.right_leg = ComputeHardwareLegKinematics(
      input.joint_position[3], input.joint_position[4],
      config.right_hip_offset_rad, config.right_knee_offset_rad,
      state->previous_right_phi, state->filtered_right_phi_rate,
      state->has_previous_right_phi, dt, config);
  state->previous_right_phi = output.right_leg.phi;
  state->filtered_right_phi_rate = output.right_leg.phi_rate;
  state->has_previous_right_phi = true;

  return output;
}

}  // namespace wheel_leg_stm32_bridge
