#ifndef WHEEL_LEG_SIMULATE_WHEEL_LEG_CONTROLLER_ORCHESTRATION_H_
#define WHEEL_LEG_SIMULATE_WHEEL_LEG_CONTROLLER_ORCHESTRATION_H_

#include <wheel_leg_common/types.hpp>

#include "lqr_k.h"
#include "pid.h"
#include "sensor.h"
#include "vmc.h"

namespace wheel_leg {

struct ControlTargets {
  double target_velocity = 0.0;
  double target_yaw_rate = 0.0;
  double target_distance = 0.0;
  double target_leg_length = 0.25;
  double target_phi = 0.0;
};

struct ControlPidSet {
  PIDInstance* leglen_pid_l = nullptr;
  PIDInstance* leglen_pid_r = nullptr;
  PIDInstance* steer_v_pid = nullptr;
  PIDInstance* anti_crash_pid = nullptr;
};

struct ControlStepOutputs {
  wheel_leg_common::ControlCommand command;
  double right_wheel_torque = 0.0;
  double left_wheel_torque = 0.0;
  double right_leg_length_force = 0.0;
  double left_leg_length_force = 0.0;
  double left_lqr_hip_torque = 0.0;
  double right_lqr_hip_torque = 0.0;
  double steer_output = 0.0;
  double anti_crash_output = 0.0;
  double swerving_speed_ff = 0.0;
};

ControlStepOutputs RunStandControlStep(
    double sim_time,
    const ControlTargets& targets,
    const RobotSensorData& sensor_data,
    const ControlPidSet& pid_set);

}  // namespace wheel_leg

#endif  // WHEEL_LEG_SIMULATE_WHEEL_LEG_CONTROLLER_ORCHESTRATION_H_
