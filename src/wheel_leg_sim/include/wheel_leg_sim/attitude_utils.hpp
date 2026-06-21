#ifndef WHEEL_LEG_SIM__ATTITUDE_UTILS_HPP_
#define WHEEL_LEG_SIM__ATTITUDE_UTILS_HPP_

#include <array>

namespace wheel_leg_sim {

struct EulerAngles {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

EulerAngles QuaternionToEuler(const std::array<double, 4>& quat);

double ComputeBaseForwardVelocity(double world_velocity_x,
                                  double world_velocity_y,
                                  double yaw);

}  // namespace wheel_leg_sim

#endif  // WHEEL_LEG_SIM__ATTITUDE_UTILS_HPP_
