#include "lqr_k.h"

#include <cmath>

namespace wheel_leg {
namespace {

std::array<double, 2> MultiplyNegativeGainByError(const LqrGain& gain,
                                                  const LqrVector& target,
                                                  const LqrVector& states) {
  std::array<double, 2> output = {};
  for (int row = 0; row < 2; ++row) {
    for (int column = 0; column < 6; ++column) {
      output[row] -= gain[row][column] * (states[column] - target[column]);
    }
  }
  return output;
}

}  // namespace

LqrGain LqrK(double leg_length) {
  const double l2 = leg_length * leg_length;
  const double l3 = l2 * leg_length;

  const double mt1 = leg_length * (-2.099093902076438e+1) +
                     l2 * 1.203494605572917e+1 -
                     l3 * 1.218320086761083e+1 -
                     5.649666477182851e-1;
  const double mt2 = leg_length * 6.294124564519359e-2 -
                     l2 * 1.375805562717833e-1 +
                     l3 * 1.211129950959015e-1 -
                     1.485210602437126e-2;
  const double mt3 = leg_length * (-3.118832132995808) -
                     l2 * 2.118543410330627 +
                     l3 * 2.236799970275547 +
                     8.259268115866775e-2;
  const double mt4 = leg_length * 8.284492912539685e-4 -
                     l2 * 1.733964755626449e-3 +
                     l3 * 1.477607626894281e-3 -
                     1.166441436130078e-4;
  const double mt5 = leg_length * (-9.708361903065298e-1) -
                     l2 * 6.034641079763349e-1 +
                     l3 * 6.056416051168247e-1 +
                     3.071134668297229e-2;
  const double mt6 = leg_length * (-4.198524071813992e-2) +
                     l2 * 1.076192380050251e-1 -
                     l3 * 1.062630300205243e-1 +
                     8.142336272615559e-3;
  const double mt7 = leg_length * (-2.228179509799943) -
                     l2 * 1.078099890193391 +
                     l3 * 1.095701566192776 +
                     5.42305467046534e-2;
  const double mt8 = leg_length * (-4.483442710876894e-2) +
                     l2 * 1.109247690365129e-1 -
                     l3 * 1.070105223487042e-1 +
                     9.215397265206336e-3;
  const double mt9 = leg_length * (-1.453866828187572e+1) +
                     l2 * 3.880266142336006e+1 -
                     l3 * 3.926742239333192e+1 +
                     2.625115289299839;
  const double mt10 = leg_length * 1.58216686312066e-2 -
                      l2 * 4.866596477478237e-2 +
                      l3 * 5.291325356527415e-2 +
                      4.084109576187354;
  const double mt11 = leg_length * (-6.615678732688394) +
                      l2 * 1.766184458431303e+1 -
                      l3 * 1.787647278189416e+1 +
                      1.201363351663543;
  const double mt12 = leg_length * 5.284888647223982e-3 -
                      l2 * 1.611685878341068e-2 +
                      l3 * 1.74275946970661e-2 +
                      1.867590067085635;

  return {{
      {{mt1, mt3, mt5, mt7, mt9, mt11}},
      {{mt2, mt4, mt6, mt8, mt10, mt12}},
  }};
}

LqrTorqueOutput CalcLqrTorque(double leg_length, const LqrVector& target,
                              const LqrVector& states) {
  const std::array<double, 2> torque =
      MultiplyNegativeGainByError(LqrK(leg_length), target, states);

  LqrTorqueOutput output = {};
  output.fly_flag = false;
  output.wheel_torque = torque[0];
  output.hip_torque = torque[1];
  output.torque_magnitude =
      std::hypot(output.wheel_torque, output.hip_torque);
  return output;
}

}  // namespace wheel_leg
