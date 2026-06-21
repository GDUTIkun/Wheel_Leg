#include "wheel_leg_control/state_assembler.hpp"

namespace wheel_leg_control {

void StateAssembler::UpdateControlState(
    const StandControlState& control_state) {
  control_state_ = control_state;
  has_control_state_ = true;
}

bool StateAssembler::HasCompleteState() const {
  return has_control_state_;
}

const StandControlState& StateAssembler::BuildState() const {
  return control_state_;
}

}  // namespace wheel_leg_control
