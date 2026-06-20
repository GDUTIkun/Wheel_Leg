#ifndef WHEEL_LEG_SIMULATE_WHEEL_LEG_SIM_ADAPTER_H_
#define WHEEL_LEG_SIMULATE_WHEEL_LEG_SIM_ADAPTER_H_

#include <cstddef>
#include <string>

#include <mujoco/mujoco.h>

#include <wheel_leg_common/types.hpp>

#include "sensor.h"

namespace wheel_leg {

struct ApplyCommandResult {
  bool accepted = false;
  bool command_was_clamped = false;
  std::size_t applied_effort_count = 0;
  std::string rejected_joint_name;
  std::string first_clamped_joint_name;
  double first_requested_effort = 0.0;
  double first_applied_effort = 0.0;
};

void ResetSimAdapterState();

RobotSensorData SampleRobotSensorData(const mjModel* m, const mjData* d);

wheel_leg_common::JointStateSample SampleJointState(
    const mjModel* m, const mjData* d);

wheel_leg_common::ImuSample SampleImu(const mjModel* m, const mjData* d);

ApplyCommandResult ApplyControlCommand(
    const mjModel* m, mjData* d,
    const wheel_leg_common::ControlCommand& command);

}  // namespace wheel_leg

#endif  // WHEEL_LEG_SIMULATE_WHEEL_LEG_SIM_ADAPTER_H_
