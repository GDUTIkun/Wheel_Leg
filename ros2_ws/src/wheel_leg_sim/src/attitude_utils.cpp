#include "wheel_leg_sim/attitude_utils.hpp"

#include <cmath>

namespace wheel_leg_sim {

EulerAngles QuaternionToEuler(const std::array<double, 4>& quat) {
  const double w = quat[0];
  const double x = quat[1];
  const double y = quat[2];
  const double z = quat[3];

  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);

  double sinp = 2.0 * (w * y - z * x);
  if (sinp > 1.0) {
    sinp = 1.0;
  } else if (sinp < -1.0) {
    sinp = -1.0;
  }

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);

  EulerAngles euler;
  euler.roll = std::atan2(sinr_cosp, cosr_cosp);
  euler.pitch = std::asin(sinp);
  euler.yaw = std::atan2(siny_cosp, cosy_cosp);
  return euler;
}

double ComputeBaseForwardVelocity(double world_velocity_x,
                                  double world_velocity_y,
                                  double yaw) {
  const double forward_x = std::cos(yaw);
  const double forward_y = std::sin(yaw);
  return world_velocity_x * forward_x + world_velocity_y * forward_y;
}

}  // namespace wheel_leg_sim
