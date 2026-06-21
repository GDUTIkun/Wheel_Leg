#ifndef WHEEL_LEG_CONTROL__LEGACY_ALGORITHMS_HPP_
#define WHEEL_LEG_CONTROL__LEGACY_ALGORITHMS_HPP_

#include <array>
#include <cstdint>

#include "wheel_leg_control/stand_algorithm_interfaces.hpp"
#include "wheel_leg_control/stand_runtime_defaults.hpp"

namespace wheel_leg_control {

class LegacyPidAlgorithm : public PidAlgorithm {
 public:
  explicit LegacyPidAlgorithm(const LegacyPidConfig& config);

  double Compute(const PidStepInput& input) override;
  void Reset(double measurement, double target);

 private:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double max_output_ = 0.0;
  double deadband_ = 0.0;
  std::uint32_t improve_ = 0;
  double integral_limit_ = 0.0;
  double coef_a_ = 0.0;
  double coef_b_ = 0.0;
  double output_lpf_rc_ = 0.0;
  double derivative_lpf_rc_ = 0.0;

  double measure_ = 0.0;
  double last_measure_ = 0.0;
  double error_ = 0.0;
  double last_error_ = 0.0;
  double last_i_term_ = 0.0;
  double p_out_ = 0.0;
  double i_out_ = 0.0;
  double d_out_ = 0.0;
  double i_term_ = 0.0;
  double output_ = 0.0;
  double last_output_ = 0.0;
  double last_d_out_ = 0.0;
  double reference_ = 0.0;
};

class LegacyLqrAlgorithm : public LqrAlgorithm {
 public:
  LqrControlOutput Compute(const LqrStepInput& input) const override;
};

class LegacyVmcAlgorithm : public VmcAlgorithm {
 public:
  VmcJointTorques Compute(const VmcStepInput& input) const override;
};

}  // namespace wheel_leg_control

#endif  // WHEEL_LEG_CONTROL__LEGACY_ALGORITHMS_HPP_
