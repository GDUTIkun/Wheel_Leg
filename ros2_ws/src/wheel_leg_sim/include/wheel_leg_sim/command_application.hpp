#ifndef WHEEL_LEG_SIM__COMMAND_APPLICATION_HPP_
#define WHEEL_LEG_SIM__COMMAND_APPLICATION_HPP_

#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <wheel_leg_common/types.hpp>

namespace wheel_leg_sim {

struct ActuatorControlRange {
  bool limited = false;
  double min_effort = 0.0;
  double max_effort = 0.0;
};

struct PreparedActuatorCommand {
  std::string actuator_name;
  double effort = 0.0;
};

struct ApplyCommandResult {
  bool accepted = false;
  bool command_was_clamped = false;
  std::size_t applied_effort_count = 0;
  std::string rejected_joint_name;
  std::string first_clamped_joint_name;
  double first_requested_effort = 0.0;
  double first_applied_effort = 0.0;
};

using ActuatorRangeLookup =
    std::function<std::optional<ActuatorControlRange>(std::string_view)>;

ApplyCommandResult PrepareActuatorCommands(
    const wheel_leg_common::ControlCommand& command,
    const ActuatorRangeLookup& range_lookup,
    std::vector<PreparedActuatorCommand>* prepared_commands);

}  // namespace wheel_leg_sim

#endif  // WHEEL_LEG_SIM__COMMAND_APPLICATION_HPP_
