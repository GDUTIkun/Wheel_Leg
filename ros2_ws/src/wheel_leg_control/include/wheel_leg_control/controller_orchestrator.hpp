#ifndef WHEEL_LEG_CONTROL__CONTROLLER_ORCHESTRATOR_HPP_
#define WHEEL_LEG_CONTROL__CONTROLLER_ORCHESTRATOR_HPP_

#include <optional>

#include <wheel_leg_common/types.hpp>

#include "wheel_leg_control/legacy_algorithms.hpp"
#include "wheel_leg_control/stand_control_types.hpp"
#include "wheel_leg_control/stand_control_runtime.hpp"

namespace wheel_leg_control {

class ControllerOrchestrator {
 public:
  ControllerOrchestrator();
  void ConfigurePidDefaults(const StandLegacyPidDefaults& defaults);

  std::optional<ControlStepOutputs> Step(
      double state_time_sec,
      double dt,
      const StandControlState& control_state);
  void SetTargets(const ControlTargets& targets);
  void SetTurnHipFeedforwardScale(double scale);
  void SetStageConfig(const StandControlStageConfig& config);
  const ControlTargets& targets() const;
  const StandControlStageConfig& stage_config() const;
  void ResetControllersForState(const StandControlState& control_state);

 private:
  LegacyPidAlgorithm leglen_pid_l_;
  LegacyPidAlgorithm leglen_pid_r_;
  LegacyPidAlgorithm steer_v_pid_;
  LegacyPidAlgorithm anti_crash_pid_;
  LegacyPidAlgorithm roll_balance_pid_;
  LegacyLqrAlgorithm lqr_algorithm_;
  LegacyVmcAlgorithm vmc_algorithm_;
  StandControlRuntime runtime_;
};

}  // namespace wheel_leg_control

#endif  // WHEEL_LEG_CONTROL__CONTROLLER_ORCHESTRATOR_HPP_
