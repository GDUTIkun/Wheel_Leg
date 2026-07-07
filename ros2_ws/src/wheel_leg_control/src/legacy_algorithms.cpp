#include "wheel_leg_control/legacy_algorithms.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

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

  const double mt1 = leg_length * (-3.219370778638849e+1) -
                     l2 * 1.694109247906909e+1 +
                     l3 * 5.432969304437776 -
                     1.934863700996925e-1;
  const double mt2 = leg_length * 3.389347924268321e-1 -
                     l2 * 7.805009842837183e-1 +
                     l3 * 7.592904561852343e-1 +
                     1.836490630296754e-2;
  const double mt3 = leg_length * (-2.725280015193897) -
                     l2 * 1.203682757077157e+1 +
                     l3 * 2.035484825047456 +
                     7.245028257539507e-2;
  const double mt4 = leg_length * 5.22068876814219e-2 -
                     l2 * 7.358291096067381e-2 +
                     l3 * 7.333074004172579e-2 +
                     2.318568155312454e-3;
  const double mt5 = leg_length * (-1.144004556019201e+1) -
                     l2 * 2.096918839154756 +
                     l3 * 4.761685377577483 +
                     4.889810944463465e-1;
  const double mt6 = leg_length * (-2.113785149095089) +
                     l2 * 5.629702049589694 -
                     l3 * 5.701359654316542 +
                     4.109136315472731e-1;
  const double mt7 = leg_length * (-8.728337176065814) -
                     l2 * 4.46614462699865 +
                     l3 * 4.558887922552494 +
                     2.186850849254006e-1;
  const double mt8 = leg_length * (-4.589626903227484e-1) +
                     l2 * 1.229404612726269 -
                     l3 * 1.247170190189442 +
                     1.069276908890913e-1;
  const double mt9 = leg_length * (-2.844404036253692e+1) +
                     l2 * 7.590199571467122e+1 -
                     l3 * 7.682129507215384e+1 +
                     5.172452606928585;
  const double mt10 = leg_length * 1.365833788876104e-1 -
                      l2 * 4.180189758311664e-1 +
                      l3 * 4.536933962936627e-1 +
                      8.040321666532769;
  const double mt11 = leg_length * (-6.517729163552137) +
                      l2 * 1.742585945320152e+1 -
                      l3 * 1.76521617059606e+1 +
                      1.212681021963326;
  const double mt12 = leg_length * 3.280394443124535e-2 -
                      l2 * 9.873090325467852e-2 +
                      l3 * 1.063362626339014e-1 +
                      1.878260401896025;

  return {{
      {{mt1, mt3, mt5, mt7, mt9, mt11}},
      {{mt2, mt4, mt6, mt8, mt10, mt12}},
  }};
}

double ClampMagnitude(double value, double limit) {
  return std::clamp(value, -limit, limit);
}

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
      d_out_ = kd_ * (measure_ - last_measure_) / dt;
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
  output.wheel_torque = torque[1];
  output.hip_torque = -torque[0];
  output.torque_magnitude =
      std::hypot(output.wheel_torque, output.hip_torque);
  return output;
}

VmcJointTorques LegacyVmcAlgorithm::Compute(
    const VmcStepInput& input) const {
  constexpr double kThighLength = 9.0 / 5.0e+1;
  constexpr double kCalfLength = 9.0 / 4.0e+1;
  const LegPolarState leg = ComputeLegPolarState(
      kThighLength,
      kCalfLength,
      input.hip_absolute,
      input.calf_absolute);
  const double leg_length =
      std::max(leg.length, std::numeric_limits<double>::epsilon());
  const double hip_minus_phi = input.hip_absolute - leg.phi;
  const double calf_minus_phi = input.calf_absolute - leg.phi;
  const double calf_projection =
      input.force * std::sin(calf_minus_phi) +
      input.torque / leg_length * std::cos(calf_minus_phi);
  const double thigh_projection =
      input.force * std::sin(hip_minus_phi) +
      input.torque / leg_length * std::cos(hip_minus_phi);

  VmcJointTorques output;
  output.hip_torque =
      kThighLength * thigh_projection + kCalfLength * calf_projection;
  output.knee_torque = kCalfLength * calf_projection;
  return output;
}

}  // namespace wheel_leg_control
