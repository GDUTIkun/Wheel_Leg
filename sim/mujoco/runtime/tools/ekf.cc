#include "ekf.h"

#include <cmath>
#include <stdexcept>

namespace wheel_leg {
namespace {

using State = ExtendedKalmanFilter::State;
using Measurement2 = ExtendedKalmanFilter::Measurement2;
using Matrix2 = ExtendedKalmanFilter::Matrix2;
using Matrix6 = ExtendedKalmanFilter::Matrix6;

Matrix6 IdentityMatrix6() {
  Matrix6 identity = {};
  for (int i = 0; i < 6; ++i) {
    identity[i][i] = 1.0;
  }
  return identity;
}

State AddState(const State& lhs, const State& rhs) {
  State result = {};
  for (int i = 0; i < 6; ++i) {
    result[i] = lhs[i] + rhs[i];
  }
  return result;
}

State ScaleState(const State& value, double scale) {
  State result = {};
  for (int i = 0; i < 6; ++i) {
    result[i] = value[i] * scale;
  }
  return result;
}

Matrix6 AddMatrix6(const Matrix6& lhs, const Matrix6& rhs) {
  Matrix6 result = {};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      result[i][j] = lhs[i][j] + rhs[i][j];
    }
  }
  return result;
}

Matrix6 SubtractMatrix6(const Matrix6& lhs, const Matrix6& rhs) {
  Matrix6 result = {};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      result[i][j] = lhs[i][j] - rhs[i][j];
    }
  }
  return result;
}

Matrix6 MultiplyMatrix6(const Matrix6& lhs, const Matrix6& rhs) {
  Matrix6 result = {};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      for (int k = 0; k < 6; ++k) {
        result[i][j] += lhs[i][k] * rhs[k][j];
      }
    }
  }
  return result;
}

std::array<std::array<double, 2>, 6> MultiplyMatrix6AndMeasurementTranspose(
    const Matrix6& lhs) {
  std::array<std::array<double, 2>, 6> result = {};
  for (int i = 0; i < 6; ++i) {
    result[i][0] = lhs[i][2];
    result[i][1] = lhs[i][3];
  }
  return result;
}

Matrix2 MultiplyMeasurementAndMatrix6AndMeasurementTranspose(
    const Matrix6& value) {
  Matrix2 result = {};
  result[0][0] = value[2][2];
  result[0][1] = value[2][3];
  result[1][0] = value[3][2];
  result[1][1] = value[3][3];
  return result;
}

Matrix6 TransposeMatrix6(const Matrix6& value) {
  Matrix6 result = {};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      result[j][i] = value[i][j];
    }
  }
  return result;
}

double Determinant2(const Matrix2& value) {
  return value[0][0] * value[1][1] - value[0][1] * value[1][0];
}

Matrix2 Inverse2(const Matrix2& value) {
  const double determinant = Determinant2(value);
  if (std::fabs(determinant) < 1e-12) {
    throw std::runtime_error("EKF measurement covariance is singular.");
  }

  Matrix2 result = {};
  result[0][0] = value[1][1] / determinant;
  result[0][1] = -value[0][1] / determinant;
  result[1][0] = -value[1][0] / determinant;
  result[1][1] = value[0][0] / determinant;
  return result;
}

Matrix2 AddMatrix2(const Matrix2& lhs, const Matrix2& rhs) {
  Matrix2 result = {};
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      result[i][j] = lhs[i][j] + rhs[i][j];
    }
  }
  return result;
}

Measurement2 ExtractMeasurementResidual(const Measurement2& measurement,
                                        const State& state) {
  return {measurement[0] - state[2], measurement[1] - state[3]};
}

double MahalanobisDistanceSquared(const Measurement2& innovation,
                                  const Matrix2& covariance) {
  const Matrix2 inverse = Inverse2(covariance);
  return innovation[0] *
             (inverse[0][0] * innovation[0] + inverse[0][1] * innovation[1]) +
         innovation[1] *
             (inverse[1][0] * innovation[0] + inverse[1][1] * innovation[1]);
}

std::array<std::array<double, 2>, 6> ComputeKalmanGain(
    const Matrix6& covariance, const Matrix2& innovation_covariance) {
  const auto covariance_h_transpose =
      MultiplyMatrix6AndMeasurementTranspose(covariance);
  const Matrix2 inverse = Inverse2(innovation_covariance);

  std::array<std::array<double, 2>, 6> result = {};
  for (int i = 0; i < 6; ++i) {
    result[i][0] = covariance_h_transpose[i][0] * inverse[0][0] +
                   covariance_h_transpose[i][1] * inverse[1][0];
    result[i][1] = covariance_h_transpose[i][0] * inverse[0][1] +
                   covariance_h_transpose[i][1] * inverse[1][1];
  }
  return result;
}

State ApplyKalmanGain(const std::array<std::array<double, 2>, 6>& kalman_gain,
                      const Measurement2& innovation) {
  State result = {};
  for (int i = 0; i < 6; ++i) {
    result[i] =
        kalman_gain[i][0] * innovation[0] + kalman_gain[i][1] * innovation[1];
  }
  return result;
}

Matrix6 MultiplyKalmanGainAndMeasurement(
    const std::array<std::array<double, 2>, 6>& kalman_gain) {
  Matrix6 result = {};
  for (int i = 0; i < 6; ++i) {
    result[i][2] = kalman_gain[i][0];
    result[i][3] = kalman_gain[i][1];
  }
  return result;
}

Matrix6 UpdateCovariance(
    const std::array<std::array<double, 2>, 6>& kalman_gain,
    const Matrix6& covariance) {
  const Matrix6 identity = IdentityMatrix6();
  const Matrix6 kh = MultiplyKalmanGainAndMeasurement(kalman_gain);
  return MultiplyMatrix6(SubtractMatrix6(identity, kh), covariance);
}

}  // namespace

ExtendedKalmanFilter::ExtendedKalmanFilter()
    : ExtendedKalmanFilter(Config{}) {}

ExtendedKalmanFilter::ExtendedKalmanFilter(
    const Config& config, DynamicsFunction dynamics_function,
    TransitionJacobianFunction transition_jacobian_function)
    : config_(config),
      dynamics_function_(dynamics_function),
      transition_jacobian_function_(transition_jacobian_function),
      p_(config.initial_covariance) {}

void ExtendedKalmanFilter::Reset() {
  x_hat_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  x_hat_minus_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  p_ = config_.initial_covariance;
}

void ExtendedKalmanFilter::SetDynamicsFunction(
    DynamicsFunction dynamics_function) {
  dynamics_function_ = dynamics_function;
}

void ExtendedKalmanFilter::SetTransitionJacobianFunction(
    TransitionJacobianFunction transition_jacobian_function) {
  transition_jacobian_function_ = transition_jacobian_function;
}

EkfEstimate ExtendedKalmanFilter::Step(double accel_x, const Measurement2& z2,
                                       const Measurement2& force,
                                       double leg_length) {
  if (dynamics_function_ == nullptr || transition_jacobian_function_ == nullptr) {
    throw std::runtime_error(
        "EKF requires dynamics and transition jacobian callbacks.");
  }

  const Measurement2 z1 = BuildStage1Measurement(accel_x);
  x_hat_minus_ = PredictState(force, leg_length);
  const Matrix6 transition = ComputeTransitionJacobian(force, leg_length);
  const Matrix6 p_minus = PredictCovariance(transition);

  FuseStage1(z1, p_minus);
  FuseStage2(z2);

  return BuildEstimate();
}

Measurement2 ExtendedKalmanFilter::BuildStage1Measurement(double accel_x) const {
  Measurement2 z1 = {0.0, 0.0};
  z1[1] = x_hat_[3] + accel_x * config_.sample_time;
  z1[0] = x_hat_[2] + z1[1] * config_.sample_time;
  return z1;
}

State ExtendedKalmanFilter::PredictState(const Measurement2& force,
                                         double leg_length) const {
  const State dx = dynamics_function_(x_hat_, force, leg_length);
  return AddState(x_hat_, ScaleState(dx, config_.sample_time));
}

Matrix6 ExtendedKalmanFilter::ComputeTransitionJacobian(
    const Measurement2& force, double leg_length) const {
  return transition_jacobian_function_(x_hat_, force, leg_length);
}

Matrix6 ExtendedKalmanFilter::PredictCovariance(
    const Matrix6& transition) const {
  return AddMatrix6(
      MultiplyMatrix6(MultiplyMatrix6(transition, p_),
                      TransposeMatrix6(transition)),
      config_.process_noise);
}

void ExtendedKalmanFilter::FuseStage1(const Measurement2& z1,
                                      const Matrix6& p_minus) {
  x_hat_ = x_hat_minus_;
  p_ = p_minus;
  FuseMeasurement(z1, config_.measurement_noise_stage1, p_minus);
}

void ExtendedKalmanFilter::FuseStage2(const Measurement2& z2) {
  FuseMeasurement(z2, config_.measurement_noise_stage2, p_);
}

void ExtendedKalmanFilter::FuseMeasurement(
    const Measurement2& measurement, const Matrix2& measurement_noise,
    const Matrix6& covariance_before_update) {
  const Matrix2 innovation_covariance = AddMatrix2(
      MultiplyMeasurementAndMatrix6AndMeasurementTranspose(
          covariance_before_update),
      measurement_noise);
  const Measurement2 innovation = ExtractMeasurementResidual(measurement, x_hat_);
  const double d2 =
      MahalanobisDistanceSquared(innovation, innovation_covariance);

  if (d2 > config_.gate_threshold) {
    return;
  }

  const auto kalman_gain =
      ComputeKalmanGain(covariance_before_update, innovation_covariance);
  x_hat_ = AddState(x_hat_, ApplyKalmanGain(kalman_gain, innovation));
  p_ = UpdateCovariance(kalman_gain, covariance_before_update);
}

EkfEstimate ExtendedKalmanFilter::BuildEstimate() const {
  EkfEstimate estimate;
  estimate.x = x_hat_[2];
  estimate.x_velocity = x_hat_[3];
  return estimate;
}

}  // namespace wheel_leg
