#include <gtest/gtest.h>

#include <string>

#include "wheel_leg_control/stand_control_pipeline.hpp"

namespace wheel_leg_control {
namespace {

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

class CountingPid final : public PidAlgorithm {
 public:
  explicit CountingPid(double output) : output_(output) {}

  double Compute(const PidStepInput& input) override {
    (void)input;
    ++calls;
    return output_;
  }

  int calls = 0;

 private:
  double output_ = 0.0;
};

class CountingLqr final : public LqrAlgorithm {
 public:
  LqrControlOutput Compute(const LqrStepInput& input) const override {
    (void)input;
    ++calls;
    return {.wheel_torque = 3.0, .hip_torque = 4.0};
  }

  mutable int calls = 0;
};

class CountingVmc final : public VmcAlgorithm {
 public:
  VmcJointTorques Compute(const VmcStepInput& input) const override {
    (void)input;
    ++calls;
    return {.hip_torque = 10.0, .knee_torque = 20.0};
  }

  mutable int calls = 0;
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

}  // namespace
}  // namespace wheel_leg_control
