#include "wheel_leg_control/legacy_algorithms.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace wheel_leg_control {
namespace {

constexpr std::uint32_t kPidIntegralLimit = 0b00000001;
constexpr std::uint32_t kPidDerivativeOnMeasurement = 0b00000010;
constexpr std::uint32_t kPidTrapezoidIntegral = 0b00000100;
constexpr std::uint32_t kPidOutputFilter = 0b00010000;
constexpr std::uint32_t kPidChangingIntegrationRate = 0b00100000;
constexpr std::uint32_t kPidDerivativeFilter = 0b01000000;

using LqrGain = std::array<std::array<double, 6>, 2>;

std::array<double, 2> MultiplyNegativeGainByError(
    const LqrGain& gain,
    const LqrStateVector& target,
    const LqrStateVector& states) {
  std::array<double, 2> output = {};
  for (int row = 0; row < 2; ++row) {
    for (int column = 0; column < 6; ++column) {
      output[row] -= gain[row][column] * (states[column] - target[column]);
    }
  }
  return output;
}

LqrGain LegacyLqrK(double leg_length) {
  const double l2 = leg_length * leg_length;
  const double l3 = l2 * leg_length;

  const double mt1 = leg_length * (-2.349316355776316e+1) -
                     l2 * 3.670399010006818 -
                     l3 * 1.398009677502502 -
                     4.548716398535465e-1;
  const double mt2 = leg_length * 2.531733709580288e-1 -
                     l2 * 5.897790396142197e-1 +
                     l3 * 5.578343373447275e-1 -
                     1.542603540670233e-2;
  const double mt3 = leg_length * (-2.842491757331281) -
                     l2 * 7.795188443728169 +
                     l3 * 2.744845527634023 +
                     7.491857269379829e-2;
  const double mt4 = leg_length * 2.782208014133796e-2 -
                     l2 * 4.807188912889567e-2 +
                     l3 * 4.634138897523876e-2 +
                     9.949627147358569e-4;
  const double mt5 = leg_length * (-3.615652807036019) -
                     l2 * 1.484558727948732 +
                     l3 * 2.628736061523324 +
                     1.3292978514407e-1;
  const double mt6 = leg_length * (-3.58890273848088e-1) +
                     l2 * 9.375372949955214e-1 -
                     l3 * 9.399878632749358e-1 +
                     7.397798544433855e-2;
  const double mt7 = leg_length * (-4.713121816995547) -
                     l2 * 2.585948987341577 +
                     l3 * 3.331086209491775 +
                     1.318122060777286e-1;
  const double mt8 = leg_length * (-2.322920714715422e-1) +
                     l2 * 5.985299521996635e-1 -
                     l3 * 5.949603502713038e-1 +
                     5.378919862781061e-2;
  const double mt9 = leg_length * (-1.44762513160952e+1) +
                     l2 * 3.863193806045109e+1 -
                     l3 * 3.910434286266879e+1 +
                     2.626937661029495;
  const double mt10 = leg_length * 3.782082377310973e-2 -
                      l2 * 1.152554743138182e-1 +
                      l3 * 1.247662033459982e-1 +
                      4.085426077362938;
  const double mt11 = leg_length * (-6.562171758008764) +
                      l2 * 1.752932223097407e+1 -
                      l3 * 1.775246714935127e+1 +
                      1.202833150163381;
  const double mt12 = leg_length * 1.499028357167176e-2 -
                      l2 * 4.484479365423555e-2 +
                      l3 * 4.808086276188632e-2 +
                      1.868113795996866;

  return {{
      {{mt1, mt3, mt5, mt7, mt9, mt11}},
      {{mt2, mt4, mt6, mt8, mt10, mt12}},
  }};
}

double ClampMagnitude(double value, double limit) {
  return std::clamp(value, -limit, limit);
}

}  // namespace

LegacyPidAlgorithm::LegacyPidAlgorithm(const LegacyPidConfig& config)
    : kp_(config.kp),
      ki_(config.ki),
      kd_(config.kd),
      max_output_(config.max_output),
      deadband_(config.deadband),
      improve_(config.improvement_flags),
      integral_limit_(config.integral_limit),
      coef_a_(config.coef_a),
      coef_b_(config.coef_b),
      output_lpf_rc_(config.output_lpf_rc),
      derivative_lpf_rc_(config.derivative_lpf_rc) {}

double LegacyPidAlgorithm::Compute(const PidStepInput& input) {
  const double dt = input.dt;
  measure_ = input.measurement;
  reference_ = input.target;
  error_ = reference_ - measure_;

  if (std::abs(error_) > deadband_) {
    p_out_ = kp_ * error_;
    i_term_ = ki_ * error_ * dt;
    d_out_ = kd_ * (error_ - last_error_) / dt;

    if (improve_ & kPidTrapezoidIntegral) {
      i_term_ = ki_ * ((error_ + last_error_) / 2.0) * dt;
    }

    if ((improve_ & kPidChangingIntegrationRate) && error_ * i_out_ > 0.0) {
      if (std::abs(error_) <= coef_b_) {
      } else if (std::abs(error_) <= (coef_a_ + coef_b_)) {
        i_term_ *= (coef_a_ - std::abs(error_) + coef_b_) / coef_a_;
      } else {
        i_term_ = 0.0;
      }
    }

    if (improve_ & kPidDerivativeOnMeasurement) {
      d_out_ = kd_ * (last_measure_ - measure_) / dt;
    }

    if (improve_ & kPidDerivativeFilter) {
      d_out_ = d_out_ * dt / (derivative_lpf_rc_ + dt) +
               last_d_out_ * derivative_lpf_rc_ / (derivative_lpf_rc_ + dt);
    }

    if (improve_ & kPidIntegralLimit) {
      const double temp_i_out = i_out_ + i_term_;
      const double temp_output = p_out_ + i_out_ + d_out_;
      if (std::abs(temp_output) > max_output_ && error_ * i_out_ > 0.0) {
        i_term_ = 0.0;
      }
      if (temp_i_out > integral_limit_) {
        i_term_ = 0.0;
        i_out_ = integral_limit_;
      } else if (temp_i_out < -integral_limit_) {
        i_term_ = 0.0;
        i_out_ = -integral_limit_;
      }
    }

    i_out_ += i_term_;
    output_ = p_out_ + i_out_ + d_out_;

    if (improve_ & kPidOutputFilter) {
      output_ = output_ * dt / (output_lpf_rc_ + dt) +
                last_output_ * output_lpf_rc_ / (output_lpf_rc_ + dt);
    }

    output_ = ClampMagnitude(output_, max_output_);
  } else {
    output_ = 0.0;
    i_term_ = 0.0;
  }

  last_measure_ = measure_;
  last_output_ = output_;
  last_d_out_ = d_out_;
  last_error_ = error_;
  last_i_term_ = i_term_;
  return output_;
}

void LegacyPidAlgorithm::Reset(double measurement, double target) {
  measure_ = measurement;
  last_measure_ = measurement;
  reference_ = target;
  error_ = target - measurement;
  last_error_ = error_;
  last_i_term_ = 0.0;
  p_out_ = 0.0;
  i_out_ = 0.0;
  d_out_ = 0.0;
  i_term_ = 0.0;
  output_ = 0.0;
  last_output_ = 0.0;
  last_d_out_ = 0.0;
}

LqrControlOutput LegacyLqrAlgorithm::Compute(
    const LqrStepInput& input) const {
  const std::array<double, 2> torque =
      MultiplyNegativeGainByError(
          LegacyLqrK(input.leg_length), input.target, input.state);

  LqrControlOutput output;
  output.fly_flag = false;
  output.wheel_torque = torque[0];
  output.hip_torque = torque[1];
  output.torque_magnitude =
      std::hypot(output.wheel_torque, output.hip_torque);
  return output;
}

VmcJointTorques LegacyVmcAlgorithm::Compute(
    const VmcStepInput& input) const {
  const double t2 = std::cos(input.phi);
  const double t3 = std::cos(input.hip_absolute);
  const double t4 = std::sin(input.phi);
  const double t5 = std::sin(input.hip_absolute);
  const double t6 = input.hip_absolute + input.calf_absolute;
  const double t7 = 1.0 / input.leg_length;
  const double t8 = std::cos(t6);
  const double t9 = std::sin(t6);
  const double t10 = t3 * (9.0 / 5.0e+1);
  const double t11 = t5 * (9.0 / 5.0e+1);
  const double t12 = -t10;
  const double t13 = -t11;
  const double t14 = t8 * (9.0 / 4.0e+1);
  const double t15 = t9 * (9.0 / 4.0e+1);
  const double t16 = t12 + t14;
  const double t17 = t13 + t15;

  VmcJointTorques output;
  output.hip_torque =
      -input.force * (t2 * t17 - t4 * t16) -
      input.torque * t7 * (t2 * t16 + t4 * t17);
  output.knee_torque =
      input.force * (t4 * t8 * (9.0 / 4.0e+1) - t2 * t15) -
      input.torque * t7 * (t2 * t14 + t4 * t15);
  return output;
}

}  // namespace wheel_leg_control
