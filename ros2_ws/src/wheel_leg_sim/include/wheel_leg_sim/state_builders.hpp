#ifndef WHEEL_LEG_SIM__STATE_BUILDERS_HPP_
#define WHEEL_LEG_SIM__STATE_BUILDERS_HPP_

#include <string_view>

#include <wheel_leg_common/types.hpp>

namespace wheel_leg_sim {

void AppendJointSample(wheel_leg_common::JointStateSample* sample,
                       std::string_view joint_name,
                       double position,
                       double velocity,
                       double effort = 0.0);

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
    double accel_z);

}  // namespace wheel_leg_sim

#endif  // WHEEL_LEG_SIM__STATE_BUILDERS_HPP_
