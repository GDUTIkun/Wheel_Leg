#ifndef WHEEL_LEG_SIMULATE_WHEEL_LEG_ROS2_BRIDGE_H_
#define WHEEL_LEG_SIMULATE_WHEEL_LEG_ROS2_BRIDGE_H_

#include <mujoco/mujoco.h>

namespace wheel_leg {

void InitializeRos2Bridge(const mjModel* m, const mjData* d);
void SpinRos2Bridge(const mjModel* m, const mjData* d);
void ApplyRos2Command(const mjModel* m, mjData* d);
void PublishRos2State(const mjModel* m, const mjData* d);
bool HasActiveRosCommandControl(const mjData* d);
bool StepRosCommandControl(const mjModel* m, mjData* d);

}  // namespace wheel_leg

#endif  // WHEEL_LEG_SIMULATE_WHEEL_LEG_ROS2_BRIDGE_H_
