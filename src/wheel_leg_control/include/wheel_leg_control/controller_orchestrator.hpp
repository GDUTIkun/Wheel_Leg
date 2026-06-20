#ifndef WHEEL_LEG_CONTROL__CONTROLLER_ORCHESTRATOR_HPP_
#define WHEEL_LEG_CONTROL__CONTROLLER_ORCHESTRATOR_HPP_

#include <optional>

#include <wheel_leg_common/types.hpp>

namespace wheel_leg_control {

class ControllerOrchestrator {
 public:
  std::optional<wheel_leg_common::ControlCommand> Step(
      const wheel_leg_common::RobotStateSnapshot& snapshot) const;
};

}  // namespace wheel_leg_control

#endif  // WHEEL_LEG_CONTROL__CONTROLLER_ORCHESTRATOR_HPP_
