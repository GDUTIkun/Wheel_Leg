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
    double hip_absolute,
    double calf_absolute,
    double previous_leg_length,
    double previous_phi,
    double filtered_length_rate,
    double filtered_phi_rate,
    bool has_previous_leg_length,
    bool has_previous_phi,
    double dt,
    const HardwareStateAssemblerConfig& config) {
  const double delta = NormalizeAngleDelta(hip_absolute - calf_absolute);
  const double knee_joint_angle = kHardwareStatePi - delta;
  const double lower_link_absolute =
      hip_absolute - kHardwareStatePi + knee_joint_angle;
  const double x = config.thigh_length * std::cos(hip_absolute) +
                   config.calf_length * std::cos(lower_link_absolute);
  const double y_clockwise = config.thigh_length * std::sin(hip_absolute) +
                             config.calf_length * std::sin(lower_link_absolute);

  HardwareLegKinematics output;
  output.hip_absolute = hip_absolute;
  output.calf_absolute = calf_absolute;
  output.leg_length = std::sqrt(x * x + y_clockwise * y_clockwise);
  output.phi = std::atan2(y_clockwise, x);

  if (dt <= 0.0 || !has_previous_phi) {
    output.phi_rate.raw = 0.0;
    output.phi_rate.filtered = 0.0;
  } else {
    output.phi_rate.raw =
        NormalizeAngleDelta(output.phi - previous_phi) / dt;
    output.phi_rate.filtered =
        config.phi_rate_low_pass_alpha * filtered_phi_rate +
        (1.0 - config.phi_rate_low_pass_alpha) * output.phi_rate.raw;
  }

  if (dt <= 0.0 || !has_previous_leg_length) {
    output.length_rate.raw = 0.0;
    output.length_rate.filtered = 0.0;
  } else {
    output.length_rate.raw = (output.leg_length - previous_leg_length) / dt;
    output.length_rate.filtered =
        config.length_rate_low_pass_alpha * filtered_length_rate +
        (1.0 - config.length_rate_low_pass_alpha) * output.length_rate.raw;
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

  output.body_velocity.raw =
      0.5 * (input.joint_velocity[2] + input.joint_velocity[5]) *
      config.wheel_radius;
  if (dt <= 0.0 || !state->has_previous_body_velocity) {
    output.body_velocity.filtered = output.body_velocity.raw;
  } else {
    output.body_velocity.filtered =
        config.body_velocity_low_pass_alpha * state->filtered_body_velocity +
        (1.0 - config.body_velocity_low_pass_alpha) * output.body_velocity.raw;
  }
  state->filtered_body_velocity = output.body_velocity.filtered;
  state->has_previous_body_velocity = true;

  if (dt > 0.0) {
    state->raw_body_distance += output.body_velocity.raw * dt;
    state->filtered_body_distance += output.body_velocity.filtered * dt;
  }
  output.body_distance.raw = state->raw_body_distance;
  output.body_distance.filtered = state->filtered_body_distance;

  output.left_leg = ComputeHardwareLegKinematics(
      input.joint_position[0], input.joint_position[1],
      state->previous_left_leg_length, state->previous_left_phi,
      state->filtered_left_length_rate, state->filtered_left_phi_rate,
      state->has_previous_left_leg_length, state->has_previous_left_phi, dt,
      config);
  state->previous_left_leg_length = output.left_leg.leg_length;
  state->previous_left_phi = output.left_leg.phi;
  state->filtered_left_length_rate = output.left_leg.length_rate.filtered;
  state->filtered_left_phi_rate = output.left_leg.phi_rate.filtered;
  state->has_previous_left_leg_length = true;
  state->has_previous_left_phi = true;

  output.right_leg = ComputeHardwareLegKinematics(
      input.joint_position[3], input.joint_position[4],
      state->previous_right_leg_length, state->previous_right_phi,
      state->filtered_right_length_rate, state->filtered_right_phi_rate,
      state->has_previous_right_leg_length, state->has_previous_right_phi, dt,
      config);
  state->previous_right_leg_length = output.right_leg.leg_length;
  state->previous_right_phi = output.right_leg.phi;
  state->filtered_right_length_rate = output.right_leg.length_rate.filtered;
  state->filtered_right_phi_rate = output.right_leg.phi_rate.filtered;
  state->has_previous_right_leg_length = true;
  state->has_previous_right_phi = true;

  return output;
}

}  // namespace wheel_leg_stm32_bridge
