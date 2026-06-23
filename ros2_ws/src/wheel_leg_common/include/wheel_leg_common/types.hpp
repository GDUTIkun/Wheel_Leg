#ifndef WHEEL_LEG_COMMON__TYPES_HPP_
#define WHEEL_LEG_COMMON__TYPES_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace wheel_leg_common {

struct TimePoint {
  std::int32_t sec = 0;
  std::uint32_t nanosec = 0;
};

struct Vector3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct JointSample {
  std::string name;
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;
};

struct JointStateSample {
  TimePoint stamp;
  std::vector<JointSample> joints;
};

struct ImuSample {
  TimePoint stamp;
  std::string frame_id;
  Quaternion orientation;
  Vector3 angular_velocity;
  Vector3 linear_acceleration;
};

struct RobotStateSnapshot {
  bool has_joint_state = false;
  bool has_imu = false;
  JointStateSample joint_state;
  ImuSample imu;
};

struct JointEffortCommand {
  std::string joint_name;
  double effort = 0.0;
};

struct ControlCommand {
  TimePoint stamp;
  std::vector<JointEffortCommand> joint_efforts;
};

inline constexpr std::array<const char*, 6> kActuatedJointNames = {{
    "left_hip",
    "left_knee",
    "left_wheel",
    "right_hip",
    "right_knee",
    "right_wheel",
}};

inline bool IsKnownJointName(const std::string& joint_name) {
  for (const char* known_name : kActuatedJointNames) {
    if (joint_name == known_name) {
      return true;
    }
  }
  return false;
}

inline double ToSeconds(const TimePoint& stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace wheel_leg_common

#endif  // WHEEL_LEG_COMMON__TYPES_HPP_
