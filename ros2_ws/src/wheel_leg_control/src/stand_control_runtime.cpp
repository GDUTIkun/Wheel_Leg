#include "wheel_leg_control/stand_control_runtime.hpp"

namespace wheel_leg_control {

StandControlRuntime::StandControlRuntime(
    StandControlCallbacks callbacks,
    ControlTargets targets,
    double turn_hip_feedforward_scale)
    : targets_(targets),
      turn_hip_feedforward_scale_(turn_hip_feedforward_scale),
      leglen_pid_l_(std::move(callbacks.leglen_pid_l)),
      leglen_pid_r_(std::move(callbacks.leglen_pid_r)),
      steer_v_pid_(std::move(callbacks.steer_v_pid)),
      anti_crash_pid_(std::move(callbacks.anti_crash_pid)),
      roll_balance_pid_(std::move(callbacks.roll_balance_pid)),
      lqr_algorithm_(std::move(callbacks.lqr_algorithm)),
      vmc_algorithm_(std::move(callbacks.vmc_algorithm)),
      algorithms_{
          .leglen_pid_l = &leglen_pid_l_,
          .leglen_pid_r = &leglen_pid_r_,
          .steer_v_pid = &steer_v_pid_,
          .anti_crash_pid = &anti_crash_pid_,
          .roll_balance_pid = &roll_balance_pid_,
          .lqr_algorithm = &lqr_algorithm_,
          .vmc_algorithm = &vmc_algorithm_,
      } {}

ControlStepOutputs StandControlRuntime::Step(
    double sim_time,
    double dt,
    const StandControlState& control_state) {
  return RunStandControlStep(
      sim_time,
      dt,
      targets_,
      control_state,
      turn_hip_feedforward_scale_,
      stage_config_,
      algorithms_);
}

}  // namespace wheel_leg_control
