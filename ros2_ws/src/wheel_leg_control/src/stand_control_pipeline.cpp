#include "wheel_leg_control/stand_control_pipeline.hpp"

#include <cmath>
#include <cstdint>

#include <wheel_leg_common/types.hpp>

namespace wheel_leg_control {
namespace {

constexpr double kLegLengthGravityCompMass = 3.99077;
constexpr double kGravityAcceleration = 9.81;

wheel_leg_common::TimePoint ToCommonTime(double sim_time) {
  wheel_leg_common::TimePoint stamp;
  stamp.sec = static_cast<std::int32_t>(sim_time);
  stamp.nanosec = static_cast<std::uint32_t>(
      (sim_time - static_cast<double>(stamp.sec)) * 1000000000.0);
  return stamp;
}

wheel_leg_common::ControlCommand BuildControlCommand(
    double sim_time,
    const VmcJointTorques& right_leg_command,
    const VmcJointTorques& left_leg_command,
    double right_wheel_torque,
    double left_wheel_torque) {
  wheel_leg_common::ControlCommand command;
  command.stamp = ToCommonTime(sim_time);
  command.joint_efforts = {
      {"right_hip", right_leg_command.hip_torque},
      {"right_knee", right_leg_command.knee_torque},
      {"left_hip", left_leg_command.hip_torque},
      {"left_knee", left_leg_command.knee_torque},
      {"right_wheel", right_wheel_torque},
      {"left_wheel", left_wheel_torque},
  };
  return command;
}

LqrStateVector BuildLqrStates(
    const LegControlState& leg,
    const BodyControlState& body_state) {
  return {{
      leg.phi,
      leg.phi_rate,
      body_state.distance,
      body_state.velocity,
      body_state.pitch,
      body_state.pitch_rate,
  }};
}

LqrStateVector BuildLqrTarget(const ControlTargets& targets) {
  return {{
      targets.target_phi,
      0.0,
      targets.target_distance,
      targets.target_velocity,
      targets.target_pitch,
      0.0,
  }};
}

}  // namespace

ControlStepOutputs RunStandControlStep(
    double sim_time,
    double dt,
    const ControlTargets& targets,
    const StandControlState& control_state,
    double turn_hip_feedforward_scale,
    const StandControlStageConfig& stage_config,
    const ControlAlgorithmSet& algorithms) {
  ControlStepOutputs outputs;
  const auto& right_leg = control_state.right_leg;
  const auto& left_leg = control_state.left_leg;

  const double u_leg_length_r =
      stage_config.enable_leg_length_pid
          ? algorithms.leglen_pid_r->Compute(
                {.measurement = right_leg.leg_length,
                 .target = targets.target_leg_length,
                 .dt = dt})
          : 0.0;
  const double u_leg_length_l =
      stage_config.enable_leg_length_pid
          ? algorithms.leglen_pid_l->Compute(
                {.measurement = left_leg.leg_length,
                 .target = targets.target_leg_length,
                 .dt = dt})
          : 0.0;
  const double leg_length_gravity_compensation =
      stage_config.enable_leg_length_pid
          ? kLegLengthGravityCompMass / 2.0 * kGravityAcceleration *
                std::cos(control_state.body.roll)
          : 0.0;
  outputs.right_leg_length_force =
      u_leg_length_r + leg_length_gravity_compensation;
  outputs.left_leg_length_force =
      u_leg_length_l + leg_length_gravity_compensation;

  const LqrStateVector lqr_target = BuildLqrTarget(targets);
  const LqrControlOutput left_lqr_output =
      stage_config.enable_lqr
          ? algorithms.lqr_algorithm->Compute(
                {.leg_length = left_leg.leg_length,
                 .target = lqr_target,
                 .state = BuildLqrStates(left_leg, control_state.body)})
          : LqrControlOutput{};
  const LqrControlOutput right_lqr_output =
      stage_config.enable_lqr
          ? algorithms.lqr_algorithm->Compute(
                {.leg_length = right_leg.leg_length,
                 .target = lqr_target,
                 .state = BuildLqrStates(right_leg, control_state.body)})
          : LqrControlOutput{};

  outputs.steer_output =
      stage_config.enable_heading_control
          ? algorithms.steer_v_pid->Compute(
                {.measurement = control_state.body.yaw_rate,
                 .target = targets.target_yaw_rate,
                 .dt = dt})
          : 0.0;
  outputs.swerving_speed_ff =
      stage_config.enable_heading_control
          ? turn_hip_feedforward_scale * outputs.steer_output
          : 0.0;
  outputs.anti_crash_output =
      stage_config.enable_anti_split
          ? algorithms.anti_crash_pid->Compute(
                {.measurement = left_leg.phi - right_leg.phi,
                 .target = 0.0,
                 .dt = dt})
          : 0.0;
  outputs.roll_balance_output =
      stage_config.enable_roll_compensation
          ? algorithms.roll_balance_pid->Compute(
                {.measurement = control_state.body.roll,
                 .target = 0.0,
                 .dt = dt})
          : 0.0;
  const double anti_crash_hip_torque =
      -outputs.anti_crash_output + outputs.swerving_speed_ff;
  outputs.left_lqr_hip_torque =
      left_lqr_output.hip_torque + anti_crash_hip_torque -
      outputs.roll_balance_output;
  outputs.right_lqr_hip_torque =
      right_lqr_output.hip_torque - anti_crash_hip_torque +
      outputs.roll_balance_output;

  const VmcJointTorques right_leg_command =
      stage_config.enable_vmc
          ? algorithms.vmc_algorithm->Compute(
                {.force = -outputs.right_leg_length_force,
                 .torque = -outputs.right_lqr_hip_torque,
                 .leg_length = right_leg.leg_length,
                 .phi = right_leg.phi,
                 .hip_absolute = right_leg.hip_absolute,
                 .calf_absolute = right_leg.calf_absolute})
          : VmcJointTorques{};
  const VmcJointTorques left_leg_command =
      stage_config.enable_vmc
          ? algorithms.vmc_algorithm->Compute(
                {.force = -outputs.left_leg_length_force,
                 .torque = -outputs.left_lqr_hip_torque,
                 .leg_length = left_leg.leg_length,
                 .phi = left_leg.phi,
                 .hip_absolute = left_leg.hip_absolute,
                 .calf_absolute = left_leg.calf_absolute})
          : VmcJointTorques{};

  outputs.right_wheel_torque = stage_config.enable_wheel_output
                                   ? right_lqr_output.wheel_torque +
                                         outputs.steer_output
                                   : 0.0;
  outputs.left_wheel_torque = stage_config.enable_wheel_output
                                  ? left_lqr_output.wheel_torque -
                                        outputs.steer_output
                                  : 0.0;
  const VmcJointTorques gated_right_leg_command{
      .hip_torque =
          stage_config.enable_hip_output ? right_leg_command.hip_torque : 0.0,
      .knee_torque =
          stage_config.enable_knee_output ? right_leg_command.knee_torque : 0.0,
  };
  const VmcJointTorques gated_left_leg_command{
      .hip_torque =
          stage_config.enable_hip_output ? left_leg_command.hip_torque : 0.0,
      .knee_torque =
          stage_config.enable_knee_output ? left_leg_command.knee_torque : 0.0,
  };
  outputs.command = BuildControlCommand(
      sim_time,
      gated_right_leg_command,
      gated_left_leg_command,
      outputs.right_wheel_torque,
      outputs.left_wheel_torque);
  return outputs;
}

}  // namespace wheel_leg_control
