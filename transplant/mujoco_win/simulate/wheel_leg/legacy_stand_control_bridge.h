#ifndef WHEEL_LEG_SIMULATE_WHEEL_LEG_LEGACY_STAND_CONTROL_BRIDGE_H_
#define WHEEL_LEG_SIMULATE_WHEEL_LEG_LEGACY_STAND_CONTROL_BRIDGE_H_

#include <wheel_leg_control/stand_control_runtime.hpp>

#include "sensor.h"

namespace wheel_leg {

struct LegacyStandStepResult {
  bool applied = false;
  double target_leg_length = 0.0;
  double current_right_leg_length = 0.0;
};

void InitializeLegacyStandControl();

bool HasLegacyStandControl();

const wheel_leg_control::ControlTargets& GetLegacyStandControlTargets();

LegacyStandStepResult ApplyLegacyStandControlStep(
    const mjModel* m, mjData* d);

wheel_leg_control::ControlStepOutputs RunLegacyStandControlStep(
    double sim_time,
    const RobotSensorData& sensor_data);

}  // namespace wheel_leg

#endif  // WHEEL_LEG_SIMULATE_WHEEL_LEG_LEGACY_STAND_CONTROL_BRIDGE_H_
