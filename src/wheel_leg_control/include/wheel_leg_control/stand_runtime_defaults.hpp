#ifndef WHEEL_LEG_CONTROL__STAND_RUNTIME_DEFAULTS_HPP_
#define WHEEL_LEG_CONTROL__STAND_RUNTIME_DEFAULTS_HPP_

#include <cstdint>

#include "wheel_leg_control/stand_control_types.hpp"

namespace wheel_leg_control {

struct LegacyPidConfig {
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double max_output = 0.0;
  double deadband = 0.0;
  std::uint32_t improvement_flags = 0;
  double integral_limit = 0.0;
  double coef_a = 0.0;
  double coef_b = 0.0;
  double output_lpf_rc = 0.0;
  double derivative_lpf_rc = 0.0;
};

struct StandLegacyPidDefaults {
  LegacyPidConfig leg_length;
  LegacyPidConfig steer_velocity;
  LegacyPidConfig anti_crash;
  LegacyPidConfig roll_balance;
};

constexpr double DegreesToRadians(double degrees) {
  return degrees * 3.14159265358979323846 / 180.0;
}

constexpr double DefaultTurnHipFeedforwardScale() {
  return 3.2;
}

inline StandLegacyPidDefaults DefaultStandLegacyPidDefaults() {
  return {
      .leg_length =
          {
              .kp = 800.0,
              .ki = 50.0,
              .kd = 30.0,
              .max_output = 5000.0,
              .deadband = 0.0001,
              .improvement_flags = 0b01100010,
              .integral_limit = 5000.0,
              .coef_a = 0.05,
              .coef_b = 0.1,
              .output_lpf_rc = 0.0,
              .derivative_lpf_rc = 0.01,
          },
      .steer_velocity =
          {
              .kp = 6.0,
              .ki = 0.8,
              .kd = 0.0,
              .max_output = 50.0,
              .deadband = 0.001,
              .improvement_flags = 0b01000001,
              .integral_limit = 50.0,
              .coef_a = 0.0,
              .coef_b = 0.0,
              .output_lpf_rc = 0.0,
              .derivative_lpf_rc = 0.01,
          },
      .anti_crash =
          {
              .kp = 20.0,
              .ki = 0.5,
              .kd = 3,
              .max_output = 10.0,
              .deadband = 0.001,
              .improvement_flags = 0b01100001,
              .integral_limit = 10.0,
              .coef_a = 0.05,
              .coef_b = 0.1,
              .output_lpf_rc = 0.0,
              .derivative_lpf_rc = 0.01,
          },
      .roll_balance =
          {
              .kp = 20.0,
              .ki = 3,
              .kd = 0.2,
              .max_output = 10.0,
              .deadband = 0.001,
              .improvement_flags = 0b01010010,
              .integral_limit = 10.0,
              .coef_a = 0.0,
              .coef_b = 0.0,
              .output_lpf_rc = 0.01,
              .derivative_lpf_rc = 0.01,
          },
  };
}

inline ControlTargets DefaultStandControlTargets() {
  ControlTargets targets;
  targets.target_leg_length = 0.25;
  targets.target_phi = DegreesToRadians(97.1);
  return targets;
}

}  // namespace wheel_leg_control

#endif  // WHEEL_LEG_CONTROL__STAND_RUNTIME_DEFAULTS_HPP_
