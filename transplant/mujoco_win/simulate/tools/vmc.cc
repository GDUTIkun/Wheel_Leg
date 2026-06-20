#include "vmc.h"

#include <cmath>

namespace wheel_leg {

VmcOutput SerialVMC(double force, double torque, double leg_length,
                    double phi, double theta1, double theta2) {
  const double t2 = std::cos(phi);
  const double t3 = std::cos(theta1);
  const double t4 = std::sin(phi);
  const double t5 = std::sin(theta1);
  const double t6 = theta1 + theta2;
  const double t7 = 1.0 / leg_length;
  const double t8 = std::cos(t6);
  const double t9 = std::sin(t6);
  const double t10 = t3 * (9.0 / 5.0e+1);
  const double t11 = t5 * (9.0 / 5.0e+1);
  const double t12 = -t10;
  const double t13 = -t11;
  const double t14 = t8 * (9.0 / 4.0e+1);
  const double t15 = t9 * (9.0 / 4.0e+1);
  const double t16 = t12 + t14;
  const double t17 = t13 + t15;

  VmcOutput output;
  output.joint1_torque =
      -force * (t2 * t17 - t4 * t16) - torque * t7 * (t2 * t16 + t4 * t17);
  output.joint2_torque =
      force * (t4 * t8 * (9.0 / 4.0e+1) - t2 * t15) -
      torque * t7 * (t2 * t14 + t4 * t15);
  return output;
}

}  // namespace wheel_leg
