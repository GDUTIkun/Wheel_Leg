#ifndef WHEEL_LEG_SIMULATE_TOOLS_LQR_K_H_
#define WHEEL_LEG_SIMULATE_TOOLS_LQR_K_H_

#include <array>

namespace wheel_leg {

using LqrGain = std::array<std::array<double, 6>, 2>;
using LqrVector = std::array<double, 6>;

struct LqrTorqueOutput {
  double wheel_torque;
  double hip_torque;
  double torque_magnitude;
  bool fly_flag;
};

// C++ version of simulate/matlab_function/LQR_K.m.
// Output order matches MATLAB reshape(..., 2, 6): K[row][column].
LqrGain LqrK(double leg_length);

// Calculates wheel and hip torque as: output = -K * (states - target).
// Vector order: [phi, phi_rate, distance, velocity, pitch, pitch_rate].
LqrTorqueOutput CalcLqrTorque(double leg_length, const LqrVector& target,
                              const LqrVector& states);

}  // namespace wheel_leg

#endif  // WHEEL_LEG_SIMULATE_TOOLS_LQR_K_H_
