#ifndef WHEEL_LEG_CONTROL__STAND_CONTROL_PIPELINE_HPP_
#define WHEEL_LEG_CONTROL__STAND_CONTROL_PIPELINE_HPP_

#include <wheel_leg_control/stand_algorithm_interfaces.hpp>
#include <wheel_leg_control/stand_control_types.hpp>

namespace wheel_leg_control {

struct ControlAlgorithmSet {
  PidAlgorithm* leglen_pid_l = nullptr;
  PidAlgorithm* leglen_pid_r = nullptr;
  PidAlgorithm* steer_v_pid = nullptr;
  PidAlgorithm* anti_crash_pid = nullptr;
  PidAlgorithm* roll_balance_pid = nullptr;
  const LqrAlgorithm* lqr_algorithm = nullptr;
  const VmcAlgorithm* vmc_algorithm = nullptr;
};

ControlStepOutputs RunStandControlStep(
    double sim_time,
    double dt,
    const ControlTargets& targets,
    const StandControlState& control_state,
    double turn_hip_feedforward_scale,
    const StandControlStageConfig& stage_config,
    const ControlAlgorithmSet& algorithms);

}  // namespace wheel_leg_control

#endif  // WHEEL_LEG_CONTROL__STAND_CONTROL_PIPELINE_HPP_
