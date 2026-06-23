#ifndef WHEEL_LEG_CONTROL__STAND_CONTROL_TYPES_HPP_
#define WHEEL_LEG_CONTROL__STAND_CONTROL_TYPES_HPP_

#include <wheel_leg_common/types.hpp>

namespace wheel_leg_control {

struct ControlTargets {
  double target_velocity = 0.0;
  double target_yaw_rate = 0.0;
  double target_distance = 0.0;
  double target_leg_length = 0.25;
  double target_phi = 0.0;
  double target_pitch = 0.0;
};

struct BodyControlState {
  double distance = 0.0;
  double velocity = 0.0;
  double roll = 0.0;
  double roll_rate = 0.0;
  double pitch = 0.0;
  double pitch_rate = 0.0;
  double yaw_rate = 0.0;
};

struct LegControlState {
  double hip_absolute = 0.0;
  double calf_absolute = 0.0;
  double leg_length = 0.0;
  double phi = 0.0;
  double phi_rate = 0.0;
};

struct StandControlState {
  BodyControlState body;
  LegControlState left_leg;
  LegControlState right_leg;
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
  double roll_balance_output = 0.0;
  double swerving_speed_ff = 0.0;
};

}  // namespace wheel_leg_control

#endif  // WHEEL_LEG_CONTROL__STAND_CONTROL_TYPES_HPP_
