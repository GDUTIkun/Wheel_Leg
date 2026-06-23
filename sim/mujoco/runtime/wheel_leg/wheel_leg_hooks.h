#ifndef WHEEL_LEG_SIMULATE_WHEEL_LEG_HOOKS_H_
#define WHEEL_LEG_SIMULATE_WHEEL_LEG_HOOKS_H_

#include <mujoco/mujoco.h>

namespace mujoco {
class Simulate;
}

namespace wheel_leg {

void SetSimulateHandle(mujoco::Simulate* sim);
void OnModelLoaded(mjModel* m, mjData* d);
void BeforeStep(mjModel* m, mjData* d);
void AfterStep(const mjModel* m, const mjData* d);

}  // namespace wheel_leg

#endif  // WHEEL_LEG_SIMULATE_WHEEL_LEG_HOOKS_H_
