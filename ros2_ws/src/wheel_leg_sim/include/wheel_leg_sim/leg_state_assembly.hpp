#ifndef WHEEL_LEG_SIM__LEG_STATE_ASSEMBLY_HPP_
#define WHEEL_LEG_SIM__LEG_STATE_ASSEMBLY_HPP_

#include "wheel_leg_sim/sensor_types.hpp"

namespace wheel_leg_sim {

LegState AssembleLeftLegState(const JointState& raw_hip,
                              const JointState& raw_knee,
                              const JointState& raw_wheel,
                              const JointState& raw_calf);

LegState AssembleRightLegState(const JointState& raw_hip,
                               const JointState& raw_knee,
                               const JointState& raw_wheel,
                               const JointState& raw_calf);

}  // namespace wheel_leg_sim

#endif  // WHEEL_LEG_SIM__LEG_STATE_ASSEMBLY_HPP_
