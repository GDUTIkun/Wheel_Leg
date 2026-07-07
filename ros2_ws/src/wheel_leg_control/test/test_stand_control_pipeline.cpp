#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "wheel_leg_control/legacy_algorithms.hpp"
#include "wheel_leg_control/stand_control_pipeline.hpp"

namespace wheel_leg_control {
namespace {

constexpr double kThighLength = 0.18;
constexpr double kCalfLength = 0.225;

double EffortForJoint(
    const wheel_leg_common::ControlCommand& command,
    const std::string& joint_name) {
  for (const auto& joint_effort : command.joint_efforts) {
    if (joint_effort.joint_name == joint_name) {
      return joint_effort.effort;
    }
  }
  return 0.0;
}

double LegLengthForAbsoluteCalf(double hip_absolute, double calf_absolute) {
  const double x =
      kThighLength * std::cos(hip_absolute) +
      kCalfLength * std::cos(calf_absolute);
  const double y =
      kThighLength * std::sin(hip_absolute) +
      kCalfLength * std::sin(calf_absolute);
  return std::hypot(x, y);
}

double PhiForAbsoluteCalf(double hip_absolute, double calf_absolute) {
  const double x =
      kThighLength * std::cos(hip_absolute) +
      kCalfLength * std::cos(calf_absolute);
  const double y =
      kThighLength * std::sin(hip_absolute) +
      kCalfLength * std::sin(calf_absolute);
  return std::atan2(y, x);
}

class CountingPid final : public PidAlgorithm {
 public:
  explicit CountingPid(double output) : output_(output) {}

  double Compute(const PidStepInput& input) override {
    last_input = input;
    ++calls;
    return output_;
  }

  int calls = 0;
  PidStepInput last_input;

 private:
  double output_ = 0.0;
};

class CountingLqr final : public LqrAlgorithm {
 public:
  LqrControlOutput Compute(const LqrStepInput& input) const override {
    last_input = input;
    ++calls;
    return {.wheel_torque = 3.0, .hip_torque = 4.0};
  }

  mutable int calls = 0;
  mutable LqrStepInput last_input;
};

class CountingVmc final : public VmcAlgorithm {
 public:
  VmcJointTorques Compute(const VmcStepInput& input) const override {
    last_input = input;
    ++calls;
    return {.hip_torque = 10.0, .knee_torque = 20.0};
  }

  mutable int calls = 0;
  mutable VmcStepInput last_input;
};

struct PipelineFixture {
  CountingPid leg_l{5.0};
  CountingPid leg_r{5.0};
  CountingPid steer{7.0};
  CountingPid anti_split{11.0};
  CountingPid roll{13.0};
  CountingLqr lqr;
  CountingVmc vmc;
  ControlAlgorithmSet algorithms{
      .leglen_pid_l = &leg_l,
      .leglen_pid_r = &leg_r,
      .steer_v_pid = &steer,
      .anti_crash_pid = &anti_split,
      .roll_balance_pid = &roll,
      .lqr_algorithm = &lqr,
      .vmc_algorithm = &vmc,
  };
  ControlTargets targets;
  StandControlState state;
};

TEST(StandControlPipelineTest, AllStagesEnabledPreservesFullCommandPath) {
  PipelineFixture fixture;

  const auto outputs = RunStandControlStep(
      1.0, 0.01, fixture.targets, fixture.state, 1.0,
      StandControlStageConfig{}, fixture.algorithms);

  EXPECT_EQ(fixture.leg_l.calls, 1);
  EXPECT_EQ(fixture.leg_r.calls, 1);
  EXPECT_EQ(fixture.lqr.calls, 2);
  EXPECT_EQ(fixture.steer.calls, 1);
  EXPECT_EQ(fixture.anti_split.calls, 1);
  EXPECT_EQ(fixture.roll.calls, 1);
  EXPECT_EQ(fixture.vmc.calls, 2);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "right_hip"), 10.0);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "right_knee"), 20.0);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "left_hip"), 10.0);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "left_knee"), 20.0);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "right_wheel"), 10.0);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "left_wheel"), -4.0);
}

TEST(StandControlPipelineTest, DisabledStagesBypassAlgorithmsAndZeroOutputs) {
  PipelineFixture fixture;
  StandControlStageConfig config;
  config.enable_vmc = false;
  config.enable_lqr = false;
  config.enable_leg_length_pid = false;
  config.enable_heading_control = false;
  config.enable_anti_split = false;
  config.enable_roll_compensation = false;
  config.enable_wheel_output = false;
  config.enable_hip_output = false;
  config.enable_knee_output = false;

  const auto outputs = RunStandControlStep(
      1.0, 0.01, fixture.targets, fixture.state, 1.0, config,
      fixture.algorithms);

  EXPECT_EQ(fixture.leg_l.calls, 0);
  EXPECT_EQ(fixture.leg_r.calls, 0);
  EXPECT_EQ(fixture.lqr.calls, 0);
  EXPECT_EQ(fixture.steer.calls, 0);
  EXPECT_EQ(fixture.anti_split.calls, 0);
  EXPECT_EQ(fixture.roll.calls, 0);
  EXPECT_EQ(fixture.vmc.calls, 0);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "right_hip"), 0.0);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "right_knee"), 0.0);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "left_hip"), 0.0);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "left_knee"), 0.0);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "right_wheel"), 0.0);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "left_wheel"), 0.0);
}

TEST(StandControlPipelineTest, OutputGatesZeroOnlySelectedActuatorGroups) {
  PipelineFixture fixture;
  StandControlStageConfig config;
  config.enable_wheel_output = false;
  config.enable_knee_output = false;

  const auto outputs = RunStandControlStep(
      1.0, 0.01, fixture.targets, fixture.state, 1.0, config,
      fixture.algorithms);

  EXPECT_EQ(fixture.vmc.calls, 2);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "right_hip"), 10.0);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "left_hip"), 10.0);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "right_knee"), 0.0);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "left_knee"), 0.0);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "right_wheel"), 0.0);
  EXPECT_DOUBLE_EQ(EffortForJoint(outputs.command, "left_wheel"), 0.0);
}

TEST(StandControlPipelineTest, LegLengthForceFeedsIntoVmcWithSignFlip) {
  PipelineFixture fixture;
  fixture.state.body.roll = 0.0;
  StandControlStageConfig config;
  config.enable_vmc = true;
  config.enable_lqr = false;
  config.enable_leg_length_pid = true;
  config.enable_heading_control = false;
  config.enable_anti_split = false;
  config.enable_roll_compensation = false;
  config.enable_wheel_output = false;

  const auto outputs = RunStandControlStep(
      1.0, 0.01, fixture.targets, fixture.state, 1.0, config,
      fixture.algorithms);

  EXPECT_DOUBLE_EQ(fixture.vmc.last_input.force, -outputs.left_leg_length_force);
}

TEST(StandControlPipelineTest, LegLengthForceIncludesGravityCompensation) {
  PipelineFixture fixture;
  fixture.state.body.roll = 0.0;

  const auto outputs = RunStandControlStep(
      1.0, 0.01, fixture.targets, fixture.state, 1.0,
      StandControlStageConfig{}, fixture.algorithms);

  EXPECT_DOUBLE_EQ(outputs.left_leg_length_pid_output, 5.0);
  EXPECT_DOUBLE_EQ(outputs.right_leg_length_pid_output, 5.0);
  EXPECT_DOUBLE_EQ(outputs.leg_length_gravity_compensation, 10.0);
  EXPECT_DOUBLE_EQ(outputs.left_leg_length_force,
                   outputs.left_leg_length_pid_output +
                       outputs.leg_length_gravity_compensation);
  EXPECT_DOUBLE_EQ(outputs.right_leg_length_force,
                   outputs.right_leg_length_pid_output +
                       outputs.leg_length_gravity_compensation);
}

TEST(StandControlPipelineTest, LqrStateOrderMatchesMatlabModel) {
  PipelineFixture fixture;
  fixture.targets.target_pitch = 0.12;
  fixture.targets.target_phi = 1.7;
  fixture.state.right_leg.leg_length = 0.31;
  fixture.state.right_leg.phi = 1.9;
  fixture.state.right_leg.phi_rate = 0.8;
  fixture.state.body.pitch = -0.2;
  fixture.state.body.pitch_rate = -0.4;
  fixture.state.body.distance = 0.5;
  fixture.state.body.velocity = -0.6;
  StandControlStageConfig config;
  config.enable_vmc = false;
  config.enable_leg_length_pid = false;
  config.enable_heading_control = false;
  config.enable_anti_split = false;
  config.enable_roll_compensation = false;

  (void)RunStandControlStep(
      1.0, 0.01, fixture.targets, fixture.state, 1.0, config,
      fixture.algorithms);

  EXPECT_DOUBLE_EQ(fixture.lqr.last_input.state[0], fixture.state.right_leg.phi);
  EXPECT_DOUBLE_EQ(fixture.lqr.last_input.state[1],
                   fixture.state.right_leg.phi_rate);
  EXPECT_DOUBLE_EQ(fixture.lqr.last_input.state[2],
                   fixture.state.body.distance);
  EXPECT_DOUBLE_EQ(fixture.lqr.last_input.state[3],
                   fixture.state.body.velocity);
  EXPECT_DOUBLE_EQ(fixture.lqr.last_input.state[4],
                   fixture.state.body.pitch);
  EXPECT_DOUBLE_EQ(fixture.lqr.last_input.state[5],
                   fixture.state.body.pitch_rate);
  EXPECT_DOUBLE_EQ(fixture.lqr.last_input.target[0], fixture.targets.target_phi);
  EXPECT_DOUBLE_EQ(fixture.lqr.last_input.target[4],
                   fixture.targets.target_pitch);
}

TEST(LegacyPidAlgorithmTest, MeasurementBelowTargetProducesPositiveOutput) {
  LegacyPidConfig config;
  config.kp = 2.0;
  config.max_output = 100.0;
  LegacyPidAlgorithm pid(config);

  const double output =
      pid.Compute({.measurement = 0.25, .target = 0.30, .dt = 0.01});

  EXPECT_GT(output, 0.0);
}

TEST(LegacyPidAlgorithmTest, MeasurementAboveTargetProducesNegativeOutput) {
  LegacyPidConfig config;
  config.kp = 2.0;
  config.max_output = 100.0;
  LegacyPidAlgorithm pid(config);

  const double output =
      pid.Compute({.measurement = 0.32, .target = 0.25, .dt = 0.01});

  EXPECT_LT(output, 0.0);
}

TEST(LegacyLqrAlgorithmTest, LegAngleErrorProducesHipRestoringTorque) {
  LegacyLqrAlgorithm lqr;
  const auto output = lqr.Compute({
      .leg_length = 0.34,
      .target = {{2.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      .state = {{1.8, 0.0, 0.0, 0.0, 0.0, 0.0}},
  });

  EXPECT_GT(output.hip_torque, 1.0);
}

TEST(LegacyLqrAlgorithmTest, ScalesPhiAndPhiRateGainsOnlyOnHipTorque) {
  LegacyLqrAlgorithm lqr;
  const auto output = lqr.Compute({
      .leg_length = 0.34,
      .target = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      .state = {{0.2, -0.3, 0.0, 0.0, 0.0, 0.0}},
  });

  EXPECT_NEAR(output.hip_torque, -1.1562960769438149, 1e-12);
  EXPECT_NEAR(output.wheel_torque, -0.010310519746996817, 1e-12);
}

TEST(LegacyLqrAlgorithmTest, AddsPhiRateDampingOnlyToHipTorque) {
  LegacyLqrAlgorithm base_lqr;
  LegacyLqrAlgorithm damped_lqr({
      .phi_rate_damping_kd = 2.5,
  });
  const LqrStepInput input{
      .leg_length = 0.34,
      .target = {{2.0, 0.0, 0.1, 0.0, 0.0, 0.0}},
      .state = {{1.8, 0.6, 0.2, -0.1, 0.05, -0.02}},
  };

  const auto base_output = base_lqr.Compute(input);
  const auto damped_output = damped_lqr.Compute(input);

  EXPECT_NEAR(damped_output.hip_torque,
              base_output.hip_torque + input.state[1] * 2.5,
              1e-12);
  EXPECT_NEAR(damped_output.wheel_torque, base_output.wheel_torque, 1e-12);
}

TEST(LegacyVmcAlgorithmTest, KneeTorqueUsesCalfAbsoluteAngle) {
  LegacyVmcAlgorithm vmc;
  const VmcStepInput input{
      .force = 10.0,
      .torque = 2.0,
      .leg_length = 0.25,
      .phi = 1.0,
      .hip_absolute = 0.3,
      .calf_absolute = 1.8,
  };

  const auto output = vmc.Compute(input);
  const double leg_length =
      LegLengthForAbsoluteCalf(input.hip_absolute, input.calf_absolute);
  const double phi =
      PhiForAbsoluteCalf(input.hip_absolute, input.calf_absolute);
  const double expected_knee_torque =
      kCalfLength *
      (input.force * std::sin(input.calf_absolute - phi) +
       input.torque / leg_length * std::cos(input.calf_absolute - phi));

  EXPECT_NEAR(output.knee_torque, expected_knee_torque, 1e-12);
}

TEST(LegacyVmcAlgorithmTest,
     ContractedLegAndPositivePhiTorqueProduceExpectedJointSigns) {
  LegacyVmcAlgorithm vmc;
  const double hip_absolute = 0.70;
  const double calf_absolute = 2.62;
  const double leg_length =
      LegLengthForAbsoluteCalf(hip_absolute, calf_absolute);
  const double phi = PhiForAbsoluteCalf(hip_absolute, calf_absolute);

  const auto length_output = vmc.Compute({
      .force = -40.0,
      .torque = 0.0,
      .leg_length = leg_length,
      .phi = phi,
      .hip_absolute = hip_absolute,
      .calf_absolute = calf_absolute,
  });
  EXPECT_LT(length_output.knee_torque, -1.0);

  const auto phi_output = vmc.Compute({
      .force = 0.0,
      .torque = 4.0,
      .leg_length = leg_length,
      .phi = phi,
      .hip_absolute = hip_absolute,
      .calf_absolute = calf_absolute,
  });
  EXPECT_GT(phi_output.hip_torque, 1.0);
}

TEST(LegacyVmcAlgorithmTest, RecomputesPolarStateFromJointAngles) {
  LegacyVmcAlgorithm vmc;
  const double requested_phi_torque = 4.0;

  const auto output = vmc.Compute({
      .force = 0.0,
      .torque = requested_phi_torque,
      .leg_length = 0.01,
      .phi = 0.0,
      .hip_absolute = 0.70,
      .calf_absolute = 2.62,
  });

  EXPECT_NEAR(output.hip_torque, requested_phi_torque, 1e-12);
}

}  // namespace
}  // namespace wheel_leg_control
