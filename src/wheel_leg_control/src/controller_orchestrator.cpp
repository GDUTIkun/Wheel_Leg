#include "wheel_leg_control/controller_orchestrator.hpp"

namespace wheel_leg_control {

std::optional<wheel_leg_common::ControlCommand> ControllerOrchestrator::Step(
    const wheel_leg_common::RobotStateSnapshot& snapshot) const {
  if (!snapshot.has_joint_state || !snapshot.has_imu ||
      snapshot.joint_state.joints.empty()) {
    return std::nullopt;
  }

  wheel_leg_common::ControlCommand command;
  command.stamp = snapshot.joint_state.stamp;
  command.joint_efforts.reserve(wheel_leg_common::kActuatedJointNames.size());

  // Iter-002 keeps the orchestration entry in ROS2 while leaving the real
  // control law migration for later steps.
  for (const char* joint_name : wheel_leg_common::kActuatedJointNames) {
    wheel_leg_common::JointEffortCommand effort;
    effort.joint_name = joint_name;
    effort.effort = 0.0;
    command.joint_efforts.push_back(effort);
  }

  return command;
}

}  // namespace wheel_leg_control
