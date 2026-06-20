#include "controller_orchestration.h"

#include <cstdint>
#include <cmath>

#include "math_utils.h"

namespace wheel_leg {
namespace {

constexpr double kSwervingSpeedFeedforwardCoef = 3.0;
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
    const VmcOutput& right_leg_command,
    const VmcOutput& left_leg_command,
    double right_wheel_torque,
    double left_wheel_torque) {
  wheel_leg_common::ControlCommand command;
  command.stamp = ToCommonTime(sim_time);
  command.joint_efforts = {
      {"right_hip", right_leg_command.joint1_torque},
      {"right_knee", right_leg_command.joint2_torque},
      {"left_hip", left_leg_command.joint1_torque},
      {"left_knee", left_leg_command.joint2_torque},
      {"right_wheel", right_wheel_torque},
      {"left_wheel", left_wheel_torque},
  };
  return command;
}

LqrVector BuildLqrStates(const LegState& leg,
                         const RobotSensorData& sensor_data) {
  return {{
      leg.kinematics.phi,
      leg.kinematics.phi_rate,
      sensor_data.base_link.distance,
      sensor_data.base_link.velocity,
      sensor_data.base_link.pitch,
      sensor_data.base_link.pitch_rate,
  }};
}

LqrVector BuildLqrTarget(const ControlTargets& targets) {
  return {{
      targets.target_phi,
      0.0,
      targets.target_distance,
      targets.target_velocity,
      0.0,
      0.0,
  }};
}

}  // namespace

ControlStepOutputs RunStandControlStep(
    double sim_time,
    const ControlTargets& targets,
    const RobotSensorData& sensor_data,
    const ControlPidSet& pid_set) {
  ControlStepOutputs outputs;
  const LegKinematics& right_leg = sensor_data.right_leg.kinematics;
  const LegKinematics& left_leg = sensor_data.left_leg.kinematics;

  const float u_leg_length_r = PIDCalculate(
      pid_set.leglen_pid_r, right_leg.leg_length, targets.target_leg_length);
  const float u_leg_length_l = PIDCalculate(
      pid_set.leglen_pid_l, left_leg.leg_length, targets.target_leg_length);
  const double leg_length_gravity_compensation =
      kLegLengthGravityCompMass / 2.0 * kGravityAcceleration *
      std::cos(sensor_data.base_link.pitch);
  outputs.right_leg_length_force =
      u_leg_length_r + leg_length_gravity_compensation;
  outputs.left_leg_length_force =
      u_leg_length_l + leg_length_gravity_compensation;

  const LqrVector lqr_target = BuildLqrTarget(targets);
  const LqrTorqueOutput left_lqr_output =
      CalcLqrTorque(left_leg.leg_length, lqr_target,
                    BuildLqrStates(sensor_data.left_leg, sensor_data));
  const LqrTorqueOutput right_lqr_output =
      CalcLqrTorque(right_leg.leg_length, lqr_target,
                    BuildLqrStates(sensor_data.right_leg, sensor_data));

  outputs.steer_output = PIDCalculate(
      pid_set.steer_v_pid,
      static_cast<float>(sensor_data.base_link.yaw_rate),
      static_cast<float>(targets.target_yaw_rate));
  outputs.swerving_speed_ff =
      kSwervingSpeedFeedforwardCoef * outputs.steer_output;
  outputs.anti_crash_output = PIDCalculate(
      pid_set.anti_crash_pid,
      static_cast<float>(left_leg.phi - right_leg.phi), 0.0f);
  const double anti_crash_hip_torque =
      -outputs.anti_crash_output + outputs.swerving_speed_ff;
  outputs.left_lqr_hip_torque =
      left_lqr_output.hip_torque + anti_crash_hip_torque;
  outputs.right_lqr_hip_torque =
      right_lqr_output.hip_torque - anti_crash_hip_torque;

  const VmcOutput right_leg_command = SerialVMC(
      -outputs.right_leg_length_force,
      -outputs.right_lqr_hip_torque,
      sensor_data.right_leg.kinematics.leg_length,
      sensor_data.right_leg.kinematics.phi,
      sensor_data.right_leg.kinematics.hip_absolute,
      sensor_data.right_leg.kinematics.calf_absolute);
  const VmcOutput left_leg_command = SerialVMC(
      -outputs.left_leg_length_force,
      -outputs.left_lqr_hip_torque,
      sensor_data.left_leg.kinematics.leg_length,
      sensor_data.left_leg.kinematics.phi,
      sensor_data.left_leg.kinematics.hip_absolute,
      sensor_data.left_leg.kinematics.calf_absolute);

  outputs.right_wheel_torque =
      right_lqr_output.wheel_torque + outputs.steer_output;
  outputs.left_wheel_torque =
      left_lqr_output.wheel_torque - outputs.steer_output;
  outputs.command = BuildControlCommand(
      sim_time,
      right_leg_command,
      left_leg_command,
      outputs.right_wheel_torque,
      outputs.left_wheel_torque);
  return outputs;
}

}  // namespace wheel_leg
