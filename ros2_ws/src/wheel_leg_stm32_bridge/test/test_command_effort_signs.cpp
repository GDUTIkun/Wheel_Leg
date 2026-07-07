#include <gtest/gtest.h>

#include "wheel_leg_stm32_bridge/command_effort_signs.hpp"

namespace wheel_leg_stm32_bridge {
namespace {

TEST(CommandEffortSignsTest, MatchesValidatedHardwarePolarity) {
  constexpr std::array<float, 6> kExpectedSigns = {
      -1.0f,
      1.0f,
      -1.0f,
      1.0f,
      1.0f,
      -1.0f,
  };

  EXPECT_EQ(kCommandEffortSigns, kExpectedSigns);
}

}  // namespace
}  // namespace wheel_leg_stm32_bridge
