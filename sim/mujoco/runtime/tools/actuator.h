#ifndef WHEEL_LEG_SIMULATE_TOOLS_ACTUATOR_H_
#define WHEEL_LEG_SIMULATE_TOOLS_ACTUATOR_H_

#include <mujoco/mujoco.h>

namespace wheel_leg {

void Set_Val(const mjModel* m, mjData* d, const char* actuator_name,
             double value);

} // namespace wheel_leg

#endif
