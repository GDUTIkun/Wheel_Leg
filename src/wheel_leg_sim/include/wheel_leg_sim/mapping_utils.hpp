#ifndef WHEEL_LEG_SIM__MAPPING_UTILS_HPP_
#define WHEEL_LEG_SIM__MAPPING_UTILS_HPP_

#include <string_view>

#include <wheel_leg_common/types.hpp>

#include "wheel_leg_sim/joint_mappings.hpp"

namespace wheel_leg_sim {

const JointMapping* FindJointMappingByRosName(std::string_view ros_name);

wheel_leg_common::TimePoint SimTimeToCommonTime(double sim_time);

}  // namespace wheel_leg_sim

#endif  // WHEEL_LEG_SIM__MAPPING_UTILS_HPP_
