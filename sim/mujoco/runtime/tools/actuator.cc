#include "actuator.h"

namespace wheel_leg {

void Set_Val(const mjModel* m, mjData* d, const char* actuator_name,
             double value) {
  int id = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name);
  if (id < 0) {
    return;
  }
  d->ctrl[id] = value;
}

}  // namespace wheel_leg
