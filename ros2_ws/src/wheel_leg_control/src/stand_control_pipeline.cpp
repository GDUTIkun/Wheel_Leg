#include "wheel_leg_control/stand_control_pipeline.hpp"

#include <cmath>
#include <cstdint>

#include <wheel_leg_common/types.hpp>

namespace wheel_leg_control {
namespace {

constexpr double kThighLength = 9.0 / 5.0e+1;
constexpr double kCalfLength = 9.0 / 4.0e+1;

struct SerialVmcJointContributions {
  VmcJointTorques total;
  VmcJointTorques force_only;
  VmcJointTorques torque_only;
};

struct LegPolarState {
  double length = 0.0;
  double phi = 0.0;
};

LegPolarState ComputeLegPolarState(
    double thigh_length,
    double calf_length,
    double hip_absolute,
    double calf_absolute) {
  const double x =
      thigh_length * std::cos(hip_absolute) +
      calf_length * std::cos(calf_absolute);
  const double y =
      thigh_length * std::sin(hip_absolute) +
      calf_length * std::sin(calf_absolute);
  return {
      .length = std::hypot(x, y),
      .phi = std::atan2(y, x),
  };
}

SerialVmcJointContributions ComputeSerialVmcJointContributions(
    double force,
    double torque,
    double /*leg_length*/,
    double /*phi*/,
    double hip_absolute,
    double calf_absolute) {
  const LegPolarState leg = ComputeLegPolarState(
      kThighLength,
      kCalfLength,
      hip_absolute,
      calf_absolute);
  const double safe_leg_length = std::max(leg.length, 1e-9);
  const double hip_minus_phi = hip_absolute - leg.phi;
  const double calf_minus_phi = calf_absolute - leg.phi;

  const double hip_force_column =
      kThighLength * std::sin(hip_minus_phi) +
      kCalfLength * std::sin(calf_minus_phi);
  const double knee_force_column =
      kCalfLength * std::sin(calf_minus_phi);
  const double hip_torque_column =
      (kThighLength * std::cos(hip_minus_phi) +
       kCalfLength * std::cos(calf_minus_phi)) /
      safe_leg_length;
  const double knee_torque_column =
      kCalfLength * std::cos(calf_minus_phi) / safe_leg_length;

  SerialVmcJointContributions output;
  output.force_only.hip_torque = hip_force_column * force;
  output.force_only.knee_torque = knee_force_column * force;
  output.torque_only.hip_torque = hip_torque_column * torque;
  output.torque_only.knee_torque = knee_torque_column * torque;
  output.total.hip_torque =
      output.force_only.hip_torque + output.torque_only.hip_torque;
  output.total.knee_torque =
      output.force_only.knee_torque + output.torque_only.knee_torque;
  return output;
}

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
  const double leg_length_gravity_compensation = 10.0;
  outputs.right_leg_length_pid_output = u_leg_length_r;
  outputs.left_leg_length_pid_output = u_leg_length_l;
  outputs.leg_length_gravity_compensation = leg_length_gravity_compensation;
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
  const SerialVmcJointContributions right_vmc_contributions =
      ComputeSerialVmcJointContributions(
          -outputs.right_leg_length_force,
          outputs.right_lqr_hip_torque,
          right_leg.leg_length,
          right_leg.phi,
          right_leg.hip_absolute,
          right_leg.calf_absolute);
  const SerialVmcJointContributions left_vmc_contributions =
      ComputeSerialVmcJointContributions(
          -outputs.left_leg_length_force,
          outputs.left_lqr_hip_torque,
          left_leg.leg_length,
          left_leg.phi,
          left_leg.hip_absolute,
          left_leg.calf_absolute);

  const VmcJointTorques right_leg_command =
      stage_config.enable_vmc
          ? algorithms.vmc_algorithm->Compute(
                {.force = -outputs.right_leg_length_force,
                 .torque = outputs.right_lqr_hip_torque,
                 .leg_length = right_leg.leg_length,
                 .phi = right_leg.phi,
                 .hip_absolute = right_leg.hip_absolute,
                 .calf_absolute = right_leg.calf_absolute})
          : VmcJointTorques{};
  const VmcJointTorques left_leg_command =
      stage_config.enable_vmc
          ? algorithms.vmc_algorithm->Compute(
                {.force = -outputs.left_leg_length_force,
                 .torque = outputs.left_lqr_hip_torque,
                 .leg_length = left_leg.leg_length,
                 .phi = left_leg.phi,
                 .hip_absolute = left_leg.hip_absolute,
                 .calf_absolute = left_leg.calf_absolute})
          : VmcJointTorques{};
  outputs.right_vmc_thigh_projection = right_leg_command.hip_torque;
  outputs.right_vmc_calf_projection = right_leg_command.knee_torque;
  outputs.left_vmc_thigh_projection = left_leg_command.hip_torque;
  outputs.left_vmc_calf_projection = left_leg_command.knee_torque;
  outputs.right_vmc_torque_column_hip =
      right_vmc_contributions.torque_only.hip_torque;
  outputs.right_vmc_torque_column_knee =
      right_vmc_contributions.torque_only.knee_torque;
  outputs.left_vmc_torque_column_hip =
      left_vmc_contributions.torque_only.hip_torque;
  outputs.left_vmc_torque_column_knee =
      left_vmc_contributions.torque_only.knee_torque;

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
