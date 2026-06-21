#ifndef WHEEL_LEG_CONTROL__STATE_ASSEMBLER_HPP_
#define WHEEL_LEG_CONTROL__STATE_ASSEMBLER_HPP_

#include "wheel_leg_control/stand_control_types.hpp"

namespace wheel_leg_control {

class StateAssembler {
 public:
  void UpdateControlState(const StandControlState& control_state);

  bool HasCompleteState() const;
  const StandControlState& BuildState() const;

 private:
  bool has_control_state_ = false;
  StandControlState control_state_;
};

}  // namespace wheel_leg_control

#endif  // WHEEL_LEG_CONTROL__STATE_ASSEMBLER_HPP_
