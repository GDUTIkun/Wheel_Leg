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

  const double mt1 = leg_length * (-3.916006030101432e+1) -
                     l2 * 3.903756202212676e+1 +
                     l3 * 4.938528551862743e+1 +
                     1.662740788654471e-1;
  const double mt2 = leg_length * 4.973015078812089 -
                     l2 * 1.135332930442043e+1 +
                     l3 * 9.842874100942772 -
                     4.658021727158618e-2;
  const double mt3 = leg_length * (-2.108045876941421) -
                     l2 * 2.050690605835361e+1 +
                     l3 * 1.149795602835442e+1 +
                     6.138225294609957e-2;
  const double mt4 = leg_length * 5.326391426644035e-1 -
                     l2 * 4.805692537360458e-1 +
                     l3 * 2.052475163019459e-1 -
                     2.716807130490347e-3;
  const double mt5 = leg_length * (-2.285223422390715e+1) +
                     l2 * 1.014284420297003e+1 +
                     l3 * 6.981864807756348 +
                     1.230857986321886;
  const double mt6 = leg_length * (-3.641158329125341) +
                     l2 * 8.601786077130965 -
                     l3 * 8.744488313316767 +
                     1.181610800942611;
  const double mt7 = leg_length * (-1.520028512969514e+1) -
                     l2 * 2.026406166024846 +
                     l3 * 1.216400449670966e+1 +
                     5.468483582718158e-1;
  const double mt8 = leg_length * 1.058042917438525e-1 -
                     l2 * 7.032182390296819e-1 +
                     l3 * 5.490757431692782e-1 +
                     3.555365310977712e-1;
  const double mt9 = leg_length * (-3.893196134477478e+1) +
                     l2 * 1.018065598099127e+2 -
                     l3 * 1.019005159254477e+2 +
                     7.267975017076902;
  const double mt10 = leg_length * 4.681117471246045 -
                      l2 * 1.423423664690649e+1 +
                      l3 * 1.536538269131528e+1 +
                      1.111290026944721e+1;
  const double mt11 = leg_length * (-5.713020129056557) +
                      l2 * 1.499930490606449e+1 -
                      l3 * 1.509217827931361e+1 +
                      1.113738603636057;
  const double mt12 = leg_length * 6.3333661621506e-1 -
                      l2 * 1.893763823431226 +
                      l3 * 2.029491882804006 +
                      1.697212891024047;

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
  output.wheel_torque = torque[0];
  output.hip_torque = torque[1];
  output.torque_magnitude =
      std::hypot(output.wheel_torque, output.hip_torque);
  return output;
}

VmcJointTorques LegacyVmcAlgorithm::Compute(
    const VmcStepInput& input) const {
  constexpr double kThighLength = 9.0 / 5.0e+1;
  constexpr double kCalfLength = 9.0 / 4.0e+1;
  const double leg_length =
      std::max(input.leg_length, std::numeric_limits<double>::epsilon());
  const double hip_minus_phi = input.hip_absolute - input.phi;
  const double calf_minus_phi = input.calf_absolute - input.phi;
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
