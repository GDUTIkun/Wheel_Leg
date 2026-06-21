#ifndef WHEEL_LEG_CONTROL__FUNCTION_ALGORITHM_ADAPTERS_HPP_
#define WHEEL_LEG_CONTROL__FUNCTION_ALGORITHM_ADAPTERS_HPP_

#include <functional>

#include "wheel_leg_control/stand_algorithm_interfaces.hpp"

namespace wheel_leg_control {

class FunctionalPidAdapter : public PidAlgorithm {
 public:
  using Function = std::function<double(const PidStepInput&)>;

  FunctionalPidAdapter() = default;
  explicit FunctionalPidAdapter(Function function)
      : function_(std::move(function)) {}

  double Compute(const PidStepInput& input) override {
    if (!function_) {
      return 0.0;
    }
    return function_(input);
  }

 private:
  Function function_;
};

class FunctionalLqrAdapter : public LqrAlgorithm {
 public:
  using Function = std::function<LqrControlOutput(const LqrStepInput&)>;

  FunctionalLqrAdapter() = default;
  explicit FunctionalLqrAdapter(Function function)
      : function_(std::move(function)) {}

  LqrControlOutput Compute(const LqrStepInput& input) const override {
    if (!function_) {
      return {};
    }
    return function_(input);
  }

 private:
  Function function_;
};

class FunctionalVmcAdapter : public VmcAlgorithm {
 public:
  using Function = std::function<VmcJointTorques(const VmcStepInput&)>;

  FunctionalVmcAdapter() = default;
  explicit FunctionalVmcAdapter(Function function)
      : function_(std::move(function)) {}

  VmcJointTorques Compute(const VmcStepInput& input) const override {
    if (!function_) {
      return {};
    }
    return function_(input);
  }

 private:
  Function function_;
};

}  // namespace wheel_leg_control

#endif  // WHEEL_LEG_CONTROL__FUNCTION_ALGORITHM_ADAPTERS_HPP_
