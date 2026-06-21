#include "wheel_leg_control/stand_control_runtime.hpp"

namespace wheel_leg_control {

StandControlRuntime::StandControlRuntime(
    StandControlCallbacks callbacks,
    ControlTargets targets)
    : targets_(targets),
      leglen_pid_l_(std::move(callbacks.leglen_pid_l)),
      leglen_pid_r_(std::move(callbacks.leglen_pid_r)),
      steer_v_pid_(std::move(callbacks.steer_v_pid)),
      anti_crash_pid_(std::move(callbacks.anti_crash_pid)),
      lqr_algorithm_(std::move(callbacks.lqr_algorithm)),
      vmc_algorithm_(std::move(callbacks.vmc_algorithm)),
      algorithms_{
          .leglen_pid_l = &leglen_pid_l_,
          .leglen_pid_r = &leglen_pid_r_,
          .steer_v_pid = &steer_v_pid_,
          .anti_crash_pid = &anti_crash_pid_,
          .lqr_algorithm = &lqr_algorithm_,
          .vmc_algorithm = &vmc_algorithm_,
      } {}

ControlStepOutputs StandControlRuntime::Step(
    double sim_time,
    double dt,
    const StandControlState& control_state) {
  return RunStandControlStep(
      sim_time, dt, targets_, control_state, algorithms_);
}

}  // namespace wheel_leg_control
