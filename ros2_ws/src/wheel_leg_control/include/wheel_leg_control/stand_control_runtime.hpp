#ifndef WHEEL_LEG_CONTROL__STAND_CONTROL_RUNTIME_HPP_
#define WHEEL_LEG_CONTROL__STAND_CONTROL_RUNTIME_HPP_

#include "wheel_leg_control/function_algorithm_adapters.hpp"
#include "wheel_leg_control/stand_control_pipeline.hpp"
#include "wheel_leg_control/stand_runtime_defaults.hpp"

namespace wheel_leg_control {

struct StandControlCallbacks {
  FunctionalPidAdapter::Function leglen_pid_l;
  FunctionalPidAdapter::Function leglen_pid_r;
  FunctionalPidAdapter::Function steer_v_pid;
  FunctionalPidAdapter::Function anti_crash_pid;
  FunctionalPidAdapter::Function roll_balance_pid;
  FunctionalLqrAdapter::Function lqr_algorithm;
  FunctionalVmcAdapter::Function vmc_algorithm;
};

class StandControlRuntime {
 public:
  explicit StandControlRuntime(
      StandControlCallbacks callbacks,
      ControlTargets targets = DefaultStandControlTargets(),
      double turn_hip_feedforward_scale = DefaultTurnHipFeedforwardScale());

  ControlStepOutputs Step(
      double sim_time,
      double dt,
      const StandControlState& control_state);

  const ControlTargets& targets() const { return targets_; }
  void set_targets(const ControlTargets& targets) { targets_ = targets; }
  double turn_hip_feedforward_scale() const {
    return turn_hip_feedforward_scale_;
  }
  void set_turn_hip_feedforward_scale(double scale) {
    turn_hip_feedforward_scale_ = scale;
  }
  const StandControlStageConfig& stage_config() const {
    return stage_config_;
  }
  void set_stage_config(const StandControlStageConfig& config) {
    stage_config_ = config;
  }

 private:
  ControlTargets targets_;
  double turn_hip_feedforward_scale_ = DefaultTurnHipFeedforwardScale();
  StandControlStageConfig stage_config_;
  FunctionalPidAdapter leglen_pid_l_;
  FunctionalPidAdapter leglen_pid_r_;
  FunctionalPidAdapter steer_v_pid_;
  FunctionalPidAdapter anti_crash_pid_;
  FunctionalPidAdapter roll_balance_pid_;
  FunctionalLqrAdapter lqr_algorithm_;
  FunctionalVmcAdapter vmc_algorithm_;
  ControlAlgorithmSet algorithms_;
};

}  // namespace wheel_leg_control

#endif  // WHEEL_LEG_CONTROL__STAND_CONTROL_RUNTIME_HPP_
