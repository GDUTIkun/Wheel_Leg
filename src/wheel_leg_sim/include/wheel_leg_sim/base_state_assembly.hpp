#ifndef WHEEL_LEG_SIM__BASE_STATE_ASSEMBLY_HPP_
#define WHEEL_LEG_SIM__BASE_STATE_ASSEMBLY_HPP_

#include "wheel_leg_sim/attitude_utils.hpp"
#include "wheel_leg_sim/sensor_types.hpp"

namespace wheel_leg_sim {

BaseLinkState AssembleBaseLinkState(const EulerAngles& euler,
                                    double roll_rate,
                                    double pitch_rate,
                                    double yaw_rate,
                                    double accel_x,
                                    double accel_y,
                                    double accel_z,
                                    double base_forward_velocity,
                                    SensorAssemblyState* state,
                                    double dt);

}  // namespace wheel_leg_sim

#endif  // WHEEL_LEG_SIM__BASE_STATE_ASSEMBLY_HPP_
