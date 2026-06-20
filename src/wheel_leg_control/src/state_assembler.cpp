#include "wheel_leg_control/state_assembler.hpp"

namespace wheel_leg_control {

void StateAssembler::UpdateJointState(
    const wheel_leg_common::JointStateSample& joint_state) {
  snapshot_.joint_state = joint_state;
  snapshot_.has_joint_state = true;
}

void StateAssembler::UpdateImu(const wheel_leg_common::ImuSample& imu) {
  snapshot_.imu = imu;
  snapshot_.has_imu = true;
}

bool StateAssembler::HasCompleteState() const {
  return snapshot_.has_joint_state && snapshot_.has_imu;
}

wheel_leg_common::RobotStateSnapshot StateAssembler::BuildSnapshot() const {
  return snapshot_;
}

}  // namespace wheel_leg_control
