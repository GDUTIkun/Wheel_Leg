#include "wheel_leg_sim/state_builders.hpp"

namespace wheel_leg_sim {

void AppendJointSample(wheel_leg_common::JointStateSample* sample,
                       std::string_view joint_name,
                       double position,
                       double velocity,
                       double effort) {
  if (!sample) {
    return;
  }

  wheel_leg_common::JointSample joint;
  joint.name = joint_name;
  joint.position = position;
  joint.velocity = velocity;
  joint.effort = effort;
  sample->joints.push_back(joint);
}

wheel_leg_common::ImuSample BuildImuSample(
    const wheel_leg_common::TimePoint& stamp,
    std::string_view frame_id,
    double qw,
    double qx,
    double qy,
    double qz,
    double gyro_x,
    double gyro_y,
    double gyro_z,
    double accel_x,
    double accel_y,
    double accel_z) {
  wheel_leg_common::ImuSample sample;
  sample.stamp = stamp;
  sample.frame_id = frame_id;
  sample.orientation.x = qx;
  sample.orientation.y = qy;
  sample.orientation.z = qz;
  sample.orientation.w = qw;
  sample.angular_velocity.x = gyro_x;
  sample.angular_velocity.y = gyro_y;
  sample.angular_velocity.z = gyro_z;
  sample.linear_acceleration.x = accel_x;
  sample.linear_acceleration.y = accel_y;
  sample.linear_acceleration.z = accel_z;
  return sample;
}

}  // namespace wheel_leg_sim
