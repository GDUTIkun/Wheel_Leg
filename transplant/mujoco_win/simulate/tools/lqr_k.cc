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

  const double mt1 = leg_length * (-2.349316355776316e+1) -
                     l2 * 3.670399010006818 -
                     l3 * 1.398009677502502 -
                     4.548716398535465e-1;
  const double mt2 = leg_length * 2.531733709580288e-1 -
                     l2 * 5.897790396142197e-1 +
                     l3 * 5.578343373447275e-1 -
                     1.542603540670233e-2;
  const double mt3 = leg_length * (-2.842491757331281) -
                     l2 * 7.795188443728169 +
                     l3 * 2.744845527634023 +
                     7.491857269379829e-2;
  const double mt4 = leg_length * 2.782208014133796e-2 -
                     l2 * 4.807188912889567e-2 +
                     l3 * 4.634138897523876e-2 +
                     9.949627147358569e-4;
  const double mt5 = leg_length * (-3.615652807036019) -
                     l2 * 1.484558727948732 +
                     l3 * 2.628736061523324 +
                     1.3292978514407e-1;
  const double mt6 = leg_length * (-3.58890273848088e-1) +
                     l2 * 9.375372949955214e-1 -
                     l3 * 9.399878632749358e-1 +
                     7.397798544433855e-2;
  const double mt7 = leg_length * (-4.713121816995547) -
                     l2 * 2.585948987341577 +
                     l3 * 3.331086209491775 +
                     1.318122060777286e-1;
  const double mt8 = leg_length * (-2.322920714715422e-1) +
                     l2 * 5.985299521996635e-1 -
                     l3 * 5.949603502713038e-1 +
                     5.378919862781061e-2;
  const double mt9 = leg_length * (-1.44762513160952e+1) +
                     l2 * 3.863193806045109e+1 -
                     l3 * 3.910434286266879e+1 +
                     2.626937661029495;
  const double mt10 = leg_length * 3.782082377310973e-2 -
                      l2 * 1.152554743138182e-1 +
                      l3 * 1.247662033459982e-1 +
                      4.085426077362938;
  const double mt11 = leg_length * (-6.562171758008764) +
                      l2 * 1.752932223097407e+1 -
                      l3 * 1.775246714935127e+1 +
                      1.202833150163381;
  const double mt12 = leg_length * 1.499028357167176e-2 -
                      l2 * 4.484479365423555e-2 +
                      l3 * 4.808086276188632e-2 +
                      1.868113795996866;

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
