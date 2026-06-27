#include <gtest/gtest.h>

#include <cmath>

#include "wheel_leg_stm32_bridge/hardware_state_assembler.hpp"

namespace wheel_leg_stm32_bridge {
namespace {

TEST(HardwareStateAssemblerTest, ComputesBodyVelocityDistanceAndLegGeometry) {
  HardwareStateAssemblerConfig config;
  config.wheel_radius = 0.5;
  config.thigh_length = 1.0;
  config.calf_length = 1.0;

  HardwareStateAssemblyInput input;
  input.joint_velocity[2] = 1.0;
  input.joint_velocity[5] = 3.0;
  HardwareStateAssemblyState state;

  const auto output = AssembleHardwareState(input, 0.1, &state, config);

  EXPECT_DOUBLE_EQ(output.body_velocity.raw, 1.0);
  EXPECT_DOUBLE_EQ(output.body_velocity.filtered, 1.0);
  EXPECT_DOUBLE_EQ(output.body_distance.raw, 0.1);
  EXPECT_DOUBLE_EQ(output.body_distance.filtered, 0.1);
  EXPECT_NEAR(output.left_leg.leg_length, 2.0, 1e-12);
  EXPECT_NEAR(output.left_leg.phi, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(output.left_leg.phi_rate.raw, 0.0);
  EXPECT_DOUBLE_EQ(output.left_leg.phi_rate.filtered, 0.0);
  EXPECT_NEAR(output.right_leg.leg_length, 2.0, 1e-12);
  EXPECT_NEAR(output.right_leg.phi, 0.0, 1e-12);
}

TEST(HardwareStateAssemblerTest, UsesStmPositionsAsWorldAbsoluteAngles) {
  HardwareStateAssemblerConfig config;
  config.thigh_length = 1.0;
  config.calf_length = 1.0;

  HardwareStateAssemblyInput input;
  input.joint_position[0] = kHardwareStatePi / 2.0;
  input.joint_position[1] = kHardwareStatePi / 2.0;
  input.joint_position[3] = 0.0;
  input.joint_position[4] = 0.0;
  HardwareStateAssemblyState state;

  const auto output = AssembleHardwareState(input, 0.1, &state, config);

  EXPECT_DOUBLE_EQ(output.left_leg.hip_absolute, input.joint_position[0]);
  EXPECT_DOUBLE_EQ(output.left_leg.calf_absolute, input.joint_position[1]);
  EXPECT_DOUBLE_EQ(output.right_leg.hip_absolute, input.joint_position[3]);
  EXPECT_DOUBLE_EQ(output.right_leg.calf_absolute, input.joint_position[4]);
  EXPECT_NEAR(output.left_leg.phi, kHardwareStatePi / 2.0, 1e-12);
  EXPECT_NEAR(output.right_leg.phi, 0.0, 1e-12);
}

TEST(HardwareStateAssemblerTest, SolvesLegGeometryFromHipMinusCalfDelta) {
  HardwareStateAssemblerConfig config;
  config.thigh_length = 1.0;
  config.calf_length = 1.0;

  HardwareStateAssemblyInput input;
  input.joint_position[0] = kHardwareStatePi / 2.0;
  input.joint_position[1] = 0.0;
  input.joint_position[3] = kHardwareStatePi / 2.0;
  input.joint_position[4] = 0.0;
  HardwareStateAssemblyState state;

  const auto output = AssembleHardwareState(input, 0.1, &state, config);

  EXPECT_NEAR(output.left_leg.leg_length, std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(output.left_leg.phi, kHardwareStatePi / 4.0, 1e-12);
  EXPECT_NEAR(output.right_leg.leg_length, output.left_leg.leg_length, 1e-12);
  EXPECT_NEAR(output.right_leg.phi, output.left_leg.phi, 1e-12);
}

TEST(HardwareStateAssemblerTest, FiltersPhiRateFromPreviousSample) {
  HardwareStateAssemblerConfig config;
  config.thigh_length = 1.0;
  config.calf_length = 1.0;
  config.phi_rate_low_pass_alpha = 0.5;

  HardwareStateAssemblyInput input;
  HardwareStateAssemblyState state;
  (void)AssembleHardwareState(input, 0.1, &state, config);

  input.joint_position[0] = kHardwareStatePi / 2.0;
  input.joint_position[1] = kHardwareStatePi / 2.0;
  input.joint_position[3] = kHardwareStatePi / 2.0;
  input.joint_position[4] = kHardwareStatePi / 2.0;
  const auto output = AssembleHardwareState(input, 0.1, &state, config);

  EXPECT_NEAR(output.left_leg.phi, kHardwareStatePi / 2.0, 1e-12);
  EXPECT_NEAR(
      output.left_leg.phi_rate.filtered, 0.5 * (kHardwareStatePi / 2.0) / 0.1,
      1e-12);
  EXPECT_NEAR(output.right_leg.phi_rate.filtered,
              output.left_leg.phi_rate.filtered, 1e-12);
}

}  // namespace
}  // namespace wheel_leg_stm32_bridge
