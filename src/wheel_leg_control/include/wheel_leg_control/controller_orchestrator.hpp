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

  std::optional<wheel_leg_common::ControlCommand> Step(
      double state_time_sec,
      double dt,
      const StandControlState& control_state);
  void SetTargets(const ControlTargets& targets);
  const ControlTargets& targets() const;
  void ResetControllersForState(const StandControlState& control_state);

 private:
  LegacyPidAlgorithm leglen_pid_l_;
  LegacyPidAlgorithm leglen_pid_r_;
  LegacyPidAlgorithm steer_v_pid_;
  LegacyPidAlgorithm anti_crash_pid_;
  LegacyLqrAlgorithm lqr_algorithm_;
  LegacyVmcAlgorithm vmc_algorithm_;
  StandControlRuntime runtime_;
};

}  // namespace wheel_leg_control

#endif  // WHEEL_LEG_CONTROL__CONTROLLER_ORCHESTRATOR_HPP_
