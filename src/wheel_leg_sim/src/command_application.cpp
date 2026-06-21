#include "wheel_leg_sim/command_application.hpp"

#include <algorithm>
#include <utility>

#include "wheel_leg_sim/mapping_utils.hpp"

namespace wheel_leg_sim {

ApplyCommandResult PrepareActuatorCommands(
    const wheel_leg_common::ControlCommand& command,
    const ActuatorRangeLookup& range_lookup,
    std::vector<PreparedActuatorCommand>* prepared_commands) {
  ApplyCommandResult result;
  if (!prepared_commands) {
    return result;
  }

  prepared_commands->clear();
  prepared_commands->reserve(command.joint_efforts.size());

  for (const auto& joint_effort : command.joint_efforts) {
    const JointMapping* mapping =
        FindJointMappingByRosName(joint_effort.joint_name);
    if (!mapping) {
      result.rejected_joint_name = joint_effort.joint_name;
      return result;
    }

    const std::optional<ActuatorControlRange> control_range =
        range_lookup(mapping->mujoco_actuator);
    if (!control_range.has_value()) {
      result.rejected_joint_name = joint_effort.joint_name;
      return result;
    }

    double effort = joint_effort.effort;
    if (control_range->limited) {
      const double requested_effort = effort;
      effort = std::clamp(effort, control_range->min_effort,
                          control_range->max_effort);
      if (effort != requested_effort) {
        if (!result.command_was_clamped) {
          result.first_clamped_joint_name = joint_effort.joint_name;
          result.first_requested_effort = requested_effort;
          result.first_applied_effort = effort;
        }
        result.command_was_clamped = true;
      }
    }

    PreparedActuatorCommand prepared_command;
    prepared_command.actuator_name = mapping->mujoco_actuator;
    prepared_command.effort = effort;
    prepared_commands->push_back(std::move(prepared_command));
  }

  result.accepted = true;
  result.applied_effort_count = prepared_commands->size();
  return result;
}

}  // namespace wheel_leg_sim
