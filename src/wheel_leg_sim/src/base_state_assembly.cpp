#include "wheel_leg_sim/base_state_assembly.hpp"

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
                                    double dt) {
  BaseLinkState base_link;
  if (!state) {
    return base_link;
  }

  if (dt > 0.0) {
    state->base_forward_distance += base_forward_velocity * dt;
  }

  base_link.distance = state->base_forward_distance;
  base_link.velocity = base_forward_velocity;
  base_link.roll = euler.roll;
  base_link.pitch = euler.pitch;
  base_link.yaw = euler.yaw;
  base_link.roll_rate = roll_rate;
  base_link.pitch_rate = pitch_rate;
  base_link.yaw_rate = yaw_rate;
  base_link.accel_x = accel_x;
  base_link.accel_y = accel_y;
  base_link.accel_z = accel_z;
  return base_link;
}

}  // namespace wheel_leg_sim
