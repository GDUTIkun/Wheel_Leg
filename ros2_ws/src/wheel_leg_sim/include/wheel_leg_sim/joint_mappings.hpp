#ifndef WHEEL_LEG_SIM__JOINT_MAPPINGS_HPP_
#define WHEEL_LEG_SIM__JOINT_MAPPINGS_HPP_

#include <array>

namespace wheel_leg_sim {

struct JointMapping {
  const char* ros_name;
  const char* mujoco_joint;
  const char* mujoco_actuator;
};

inline constexpr std::array<JointMapping, 6> kJointMappings = {{
    {"left_hip", "left_hip_joint", "left_hip_motor"},
    {"left_knee", "left_knee_joint", "left_knee_motor"},
    {"left_wheel", "left_wheel_joint", "left_wheel_motor"},
    {"right_hip", "right_hip_joint", "right_hip_motor"},
    {"right_knee", "right_knee_joint", "right_knee_motor"},
    {"right_wheel", "right_wheel_joint", "right_wheel_motor"},
}};

}  // namespace wheel_leg_sim

#endif  // WHEEL_LEG_SIM__JOINT_MAPPINGS_HPP_
