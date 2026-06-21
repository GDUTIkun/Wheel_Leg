#ifndef WHEEL_LEG_SIM__LEG_KINEMATICS_HPP_
#define WHEEL_LEG_SIM__LEG_KINEMATICS_HPP_

#include "wheel_leg_sim/sensor_types.hpp"

namespace wheel_leg_sim {

double NormalizeAngleDelta(double angle_delta);

void UpdatePhiRate(LegKinematics* kinematics,
                   double* previous_phi,
                   double* filtered_phi_rate,
                   bool* has_previous_phi,
                   double dt,
                   double low_pass_alpha = 0.95);

LegKinematics ComputeLegKinematics(double hip_absolute,
                                   double knee_absolute,
                                   double calf_absolute);

}  // namespace wheel_leg_sim

#endif  // WHEEL_LEG_SIM__LEG_KINEMATICS_HPP_
