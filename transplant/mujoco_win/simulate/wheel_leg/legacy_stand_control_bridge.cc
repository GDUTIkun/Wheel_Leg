#include "legacy_stand_control_bridge.h"

#include <memory>
#include <optional>
#include <vector>

#include <wheel_leg_common/types.hpp>
#include <wheel_leg_sim/command_application.hpp>
#include <wheel_leg_sim/control_state_bridge.hpp>
#include <wheel_leg_control/legacy_algorithms.hpp>
#include <wheel_leg_control/stand_runtime_defaults.hpp>

#include "sensor.h"

namespace wheel_leg {
namespace {

constexpr double kLegacyStandControlDtSec = 0.002;

std::unique_ptr<wheel_leg_control::LegacyPidAlgorithm> g_leglen_pid_l;
std::unique_ptr<wheel_leg_control::LegacyPidAlgorithm> g_leglen_pid_r;
std::unique_ptr<wheel_leg_control::LegacyPidAlgorithm> g_steer_v_pid;
std::unique_ptr<wheel_leg_control::LegacyPidAlgorithm> g_anti_crash_pid;
std::unique_ptr<wheel_leg_control::LegacyLqrAlgorithm> g_lqr_algorithm;
std::unique_ptr<wheel_leg_control::LegacyVmcAlgorithm> g_vmc_algorithm;
std::unique_ptr<wheel_leg_control::StandControlRuntime> g_stand_control_runtime;

wheel_leg_sim::ApplyCommandResult ApplyControlCommand(
    const mjModel* m, mjData* d,
    const wheel_leg_common::ControlCommand& command) {
  if (!m || !d) {
    return wheel_leg_sim::ApplyCommandResult();
  }

  std::vector<wheel_leg_sim::PreparedActuatorCommand> prepared_commands;
  const auto result = wheel_leg_sim::PrepareActuatorCommands(
      command,
      [m](std::string_view actuator_name)
          -> std::optional<wheel_leg_sim::ActuatorControlRange> {
        const int actuator_id =
            mj_name2id(m, mjOBJ_ACTUATOR, std::string(actuator_name).c_str());
        if (actuator_id < 0) {
          return std::nullopt;
        }

        wheel_leg_sim::ActuatorControlRange range;
        range.limited = m->actuator_ctrllimited[actuator_id];
        if (range.limited) {
          range.min_effort = m->actuator_ctrlrange[2 * actuator_id];
          range.max_effort = m->actuator_ctrlrange[2 * actuator_id + 1];
        }
        return range;
      },
      &prepared_commands);
  if (!result.accepted) {
    return result;
  }

  for (const auto& prepared_command : prepared_commands) {
    const int actuator_id =
        mj_name2id(m, mjOBJ_ACTUATOR, prepared_command.actuator_name.c_str());
    if (actuator_id < 0) {
      wheel_leg_sim::ApplyCommandResult failed_result = result;
      failed_result.accepted = false;
      failed_result.rejected_joint_name = prepared_command.actuator_name;
      return failed_result;
    }
    d->ctrl[actuator_id] = prepared_command.effort;
  }

  return result;
}

}  // namespace

void InitializeLegacyStandControl() {
  const wheel_leg_control::StandLegacyPidDefaults pid_defaults =
      wheel_leg_control::DefaultStandLegacyPidDefaults();
  g_leglen_pid_l =
      std::make_unique<wheel_leg_control::LegacyPidAlgorithm>(
          pid_defaults.leg_length);
  g_leglen_pid_r =
      std::make_unique<wheel_leg_control::LegacyPidAlgorithm>(
          pid_defaults.leg_length);
  g_steer_v_pid =
      std::make_unique<wheel_leg_control::LegacyPidAlgorithm>(
          pid_defaults.steer_velocity);
  g_anti_crash_pid =
      std::make_unique<wheel_leg_control::LegacyPidAlgorithm>(
          pid_defaults.anti_crash);
  g_lqr_algorithm =
      std::make_unique<wheel_leg_control::LegacyLqrAlgorithm>();
  g_vmc_algorithm =
      std::make_unique<wheel_leg_control::LegacyVmcAlgorithm>();

  g_stand_control_runtime =
      std::make_unique<wheel_leg_control::StandControlRuntime>(
          wheel_leg_control::StandControlCallbacks{
              .leglen_pid_l =
                  [](const wheel_leg_control::PidStepInput& input) {
                    return g_leglen_pid_l->Compute(input);
                  },
              .leglen_pid_r =
                  [](const wheel_leg_control::PidStepInput& input) {
                    return g_leglen_pid_r->Compute(input);
                  },
              .steer_v_pid =
                  [](const wheel_leg_control::PidStepInput& input) {
                    return g_steer_v_pid->Compute(input);
                  },
              .anti_crash_pid =
                  [](const wheel_leg_control::PidStepInput& input) {
                    return g_anti_crash_pid->Compute(input);
                  },
              .lqr_algorithm =
                  [](const wheel_leg_control::LqrStepInput& input) {
                    return g_lqr_algorithm->Compute(input);
                  },
              .vmc_algorithm =
                  [](const wheel_leg_control::VmcStepInput& input) {
                    return g_vmc_algorithm->Compute(input);
                  },
          });
}

bool HasLegacyStandControl() {
  return g_stand_control_runtime != nullptr;
}

const wheel_leg_control::ControlTargets& GetLegacyStandControlTargets() {
  return g_stand_control_runtime->targets();
}

LegacyStandStepResult ApplyLegacyStandControlStep(
    const mjModel* m, mjData* d) {
  LegacyStandStepResult result;
  if (!m || !d || !HasLegacyStandControl()) {
    return result;
  }

  const RobotSensorData sensor_data = AssembleSensorData(m, d);
  const auto& targets = g_stand_control_runtime->targets();
  const wheel_leg_control::ControlStepOutputs control_outputs =
      RunLegacyStandControlStep(d->time, sensor_data);
  ApplyControlCommand(m, d, control_outputs.command);

  result.applied = true;
  result.target_leg_length = targets.target_leg_length;
  result.current_right_leg_length = sensor_data.right_leg.kinematics.leg_length;
  return result;
}

wheel_leg_control::ControlStepOutputs RunLegacyStandControlStep(
    double sim_time,
    const RobotSensorData& sensor_data) {
  return g_stand_control_runtime->Step(
      sim_time,
      kLegacyStandControlDtSec,
      wheel_leg_sim::BuildStandControlState(sensor_data));
}

}  // namespace wheel_leg
