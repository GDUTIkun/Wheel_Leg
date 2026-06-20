#ifndef WHEEL_LEG_CONTROL__STATE_ASSEMBLER_HPP_
#define WHEEL_LEG_CONTROL__STATE_ASSEMBLER_HPP_

#include <wheel_leg_common/types.hpp>

namespace wheel_leg_control {

class StateAssembler {
 public:
  void UpdateJointState(const wheel_leg_common::JointStateSample& joint_state);
  void UpdateImu(const wheel_leg_common::ImuSample& imu);

  bool HasCompleteState() const;
  wheel_leg_common::RobotStateSnapshot BuildSnapshot() const;

 private:
  wheel_leg_common::RobotStateSnapshot snapshot_;
};

}  // namespace wheel_leg_control

#endif  // WHEEL_LEG_CONTROL__STATE_ASSEMBLER_HPP_
