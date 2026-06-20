#ifndef WHEEL_LEG_SIMULATE_TOOLS_EKF_H_
#define WHEEL_LEG_SIMULATE_TOOLS_EKF_H_

#include <array>

namespace wheel_leg {

struct EkfEstimate {
  double x;
  double x_velocity;
};

class ExtendedKalmanFilter {
 public:
  using State = std::array<double, 6>;
  using Measurement2 = std::array<double, 2>;
  using Matrix2 = std::array<std::array<double, 2>, 2>;
  using Matrix6 = std::array<std::array<double, 6>, 6>;
  using DynamicsFunction = State (*)(const State& state,
                                     const Measurement2& force,
                                     double leg_length);
  using TransitionJacobianFunction = Matrix6 (*)(const State& state,
                                                 const Measurement2& force,
                                                 double leg_length);

  struct Config {
    double sample_time = 0.01;
    double gate_threshold = 4.0;
    Matrix6 initial_covariance = {{
        {{0.1, 0.0, 0.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.1, 0.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.1, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0, 0.1, 0.0, 0.0}},
        {{0.0, 0.0, 0.0, 0.0, 0.1, 0.0}},
        {{0.0, 0.0, 0.0, 0.0, 0.0, 0.1}},
    }};
    Matrix6 process_noise = {{
        {{1e-3, 0.0, 0.0, 0.0, 0.0, 0.0}},
        {{0.0, 1e-3, 0.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 1e-3, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0, 1e-3, 0.0, 0.0}},
        {{0.0, 0.0, 0.0, 0.0, 1e-3, 0.0}},
        {{0.0, 0.0, 0.0, 0.0, 0.0, 1e-3}},
    }};
    Matrix2 measurement_noise_stage1 = {{
        {{1e-2, 0.0}},
        {{0.0, 1e-2}},
    }};
    Matrix2 measurement_noise_stage2 = {{
        {{1e-1, 0.0}},
        {{0.0, 1e-1}},
    }};
  };

  ExtendedKalmanFilter();
  explicit ExtendedKalmanFilter(
      const Config& config,
      DynamicsFunction dynamics_function = nullptr,
      TransitionJacobianFunction transition_jacobian_function = nullptr);

  void Reset();
  void SetDynamicsFunction(DynamicsFunction dynamics_function);
  void SetTransitionJacobianFunction(
      TransitionJacobianFunction transition_jacobian_function);

  EkfEstimate Step(double accel_x, const Measurement2& z2,
                   const Measurement2& force, double leg_length);

  const State& state() const { return x_hat_; }
  const State& predicted_state() const { return x_hat_minus_; }
  const Matrix6& covariance() const { return p_; }

 private:
  Measurement2 BuildStage1Measurement(double accel_x) const;
  State PredictState(const Measurement2& force, double leg_length) const;
  Matrix6 ComputeTransitionJacobian(const Measurement2& force,
                                    double leg_length) const;
  Matrix6 PredictCovariance(const Matrix6& transition) const;
  void FuseStage1(const Measurement2& z1, const Matrix6& p_minus);
  void FuseStage2(const Measurement2& z2);
  void FuseMeasurement(const Measurement2& measurement,
                       const Matrix2& measurement_noise,
                       const Matrix6& covariance_before_update);
  EkfEstimate BuildEstimate() const;

  Config config_;
  DynamicsFunction dynamics_function_;
  TransitionJacobianFunction transition_jacobian_function_;
  State x_hat_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  State x_hat_minus_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Matrix6 p_ = {};
};

}  // namespace wheel_leg

#endif  // WHEEL_LEG_SIMULATE_TOOLS_EKF_H_
