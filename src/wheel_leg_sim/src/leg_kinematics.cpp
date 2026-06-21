#include "wheel_leg_sim/leg_kinematics.hpp"

#include <cmath>

namespace wheel_leg_sim {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kL1 = 0.18;
constexpr double kL2 = 0.225;

}  // namespace

double NormalizeAngleDelta(double angle_delta) {
  while (angle_delta > kPi) {
    angle_delta -= 2.0 * kPi;
  }
  while (angle_delta < -kPi) {
    angle_delta += 2.0 * kPi;
  }
  return angle_delta;
}

void UpdatePhiRate(LegKinematics* kinematics,
                   double* previous_phi,
                   double* filtered_phi_rate,
                   bool* has_previous_phi,
                   double dt,
                   double low_pass_alpha) {
  if (!kinematics || !previous_phi || !filtered_phi_rate || !has_previous_phi) {
    return;
  }

  double raw_phi_rate = 0.0;
  if (dt <= 0.0 || !*has_previous_phi) {
    *filtered_phi_rate = 0.0;
  } else {
    raw_phi_rate = NormalizeAngleDelta(kinematics->phi - *previous_phi) / dt;
    *filtered_phi_rate =
        low_pass_alpha * *filtered_phi_rate +
        (1.0 - low_pass_alpha) * raw_phi_rate;
  }

  kinematics->phi_rate = *filtered_phi_rate;
  *previous_phi = kinematics->phi;
  *has_previous_phi = true;
}

LegKinematics ComputeLegKinematics(double hip_absolute,
                                   double knee_absolute,
                                   double calf_absolute) {
  const double theta_l2 = hip_absolute + calf_absolute - kPi;
  const double x = kL1 * std::cos(hip_absolute) + kL2 * std::cos(theta_l2);
  const double y_clockwise =
      kL1 * std::sin(hip_absolute) + kL2 * std::sin(theta_l2);

  LegKinematics output;
  output.hip_absolute = hip_absolute;
  output.knee_absolute = knee_absolute;
  output.calf_absolute = calf_absolute;
  output.leg_length = std::sqrt(x * x + y_clockwise * y_clockwise);
  output.phi = std::atan2(y_clockwise, x);
  output.phi_rate = 0.0;
  return output;
}

}  // namespace wheel_leg_sim
