#include "wheel_leg_control/controller_orchestrator.hpp"

namespace wheel_leg_control {

ControllerOrchestrator::ControllerOrchestrator()
    : leglen_pid_l_(DefaultStandLegacyPidDefaults().leg_length),
      leglen_pid_r_(DefaultStandLegacyPidDefaults().leg_length),
      steer_v_pid_(DefaultStandLegacyPidDefaults().steer_velocity),
      anti_crash_pid_(DefaultStandLegacyPidDefaults().anti_crash),
      runtime_(
          StandControlCallbacks{
              .leglen_pid_l =
                  [this](const PidStepInput& input) {
                    return leglen_pid_l_.Compute(input);
                  },
              .leglen_pid_r =
                  [this](const PidStepInput& input) {
                    return leglen_pid_r_.Compute(input);
                  },
              .steer_v_pid =
                  [this](const PidStepInput& input) {
                    return steer_v_pid_.Compute(input);
                  },
              .anti_crash_pid =
                  [this](const PidStepInput& input) {
                    return anti_crash_pid_.Compute(input);
                  },
              .lqr_algorithm =
                  [this](const LqrStepInput& input) {
                    return lqr_algorithm_.Compute(input);
                  },
              .vmc_algorithm =
                  [this](const VmcStepInput& input) {
                    return vmc_algorithm_.Compute(input);
                  },
          },
          DefaultStandControlTargets()) {}

std::optional<wheel_leg_common::ControlCommand> ControllerOrchestrator::Step(
    double state_time_sec,
    double dt,
    const StandControlState& control_state) {
  return runtime_.Step(state_time_sec, dt, control_state).command;
}

void ControllerOrchestrator::SetTargets(const ControlTargets& targets) {
  runtime_.set_targets(targets);
}

const ControlTargets& ControllerOrchestrator::targets() const {
  return runtime_.targets();
}

void ControllerOrchestrator::ResetControllersForState(
    const StandControlState& control_state) {
  const auto& targets = runtime_.targets();
  leglen_pid_l_.Reset(
      control_state.left_leg.leg_length, targets.target_leg_length);
  leglen_pid_r_.Reset(
      control_state.right_leg.leg_length, targets.target_leg_length);
  steer_v_pid_.Reset(
      control_state.body.yaw_rate, targets.target_yaw_rate);
  anti_crash_pid_.Reset(
      control_state.left_leg.phi - control_state.right_leg.phi, 0.0);
}

}  // namespace wheel_leg_control
