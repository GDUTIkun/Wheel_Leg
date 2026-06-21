#include "wheel_leg_sim/mapping_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace wheel_leg_sim {

const JointMapping* FindJointMappingByRosName(std::string_view ros_name) {
  const auto iter = std::find_if(
      kJointMappings.begin(), kJointMappings.end(),
      [ros_name](const JointMapping& mapping) {
        return ros_name == mapping.ros_name;
      });
  return iter == kJointMappings.end() ? nullptr : &(*iter);
}

wheel_leg_common::TimePoint SimTimeToCommonTime(double sim_time) {
  wheel_leg_common::TimePoint stamp;
  const double clamped_time = std::max(0.0, sim_time);
  stamp.sec = static_cast<std::int32_t>(std::floor(clamped_time));
  stamp.nanosec = static_cast<std::uint32_t>(
      (clamped_time - static_cast<double>(stamp.sec)) * 1000000000.0);
  return stamp;
}

}  // namespace wheel_leg_sim
