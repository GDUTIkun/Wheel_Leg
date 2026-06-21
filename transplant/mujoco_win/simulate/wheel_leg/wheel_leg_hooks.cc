#include "wheel_leg_hooks.h"
#include "legacy_stand_control_bridge.h"
#include "simulate.h"
#include "ros2_bridge.h"
#include "sensor.h"

#include <iostream>

namespace wheel_leg {
void SetSimulateHandle(mujoco::Simulate* sim) {
  (void)sim;
}

void OnModelLoaded(mjModel* m, mjData* d) {
  std::cout << "wheel_leg simulate ready: nq=" << m->nq << ", nv=" << m->nv
            << ", nu=" << m->nu << std::endl;

  // Let the official simulate UI sliders start from zero torque.
  for (int i = 0; i < m->nu; ++i) {
    d->ctrl[i] = 0.0;
  }

  InitializeLegacyStandControl();
  ResetSensorAssemblyState();

  std::cout << "Open the right-side Control panel in simulate UI and drag the"
            << " actuator sliders to change torque." << std::endl;
  std::cout << "Fixed stand target: phi=90deg, distance=0m, velocity=0m/s, "
            << "yaw-rate=0deg/s." << std::endl;
  InitializeRos2Bridge(m, d);
}

void BeforeStep(mjModel* m, mjData* d) {
  if (StepRosCommandControl(m, d)) {
    return;
  }
  if (!HasLegacyStandControl()) {
    return;
  }
  ApplyLegacyStandControlStep(m, d);
  ApplyRos2Command(m, d);
  (void)m;
  (void)d;
}

void AfterStep(const mjModel* m, const mjData* d) {
  PublishRos2State(m, d);
}

}  // namespace wheel_leg
