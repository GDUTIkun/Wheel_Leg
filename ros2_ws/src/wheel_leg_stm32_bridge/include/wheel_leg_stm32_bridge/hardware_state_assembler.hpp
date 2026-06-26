#ifndef WHEEL_LEG_STM32_BRIDGE__HARDWARE_STATE_ASSEMBLER_HPP_
#define WHEEL_LEG_STM32_BRIDGE__HARDWARE_STATE_ASSEMBLER_HPP_

#include <array>

namespace wheel_leg_stm32_bridge {

inline constexpr double kHardwareStatePi = 3.14159265358979323846;

struct HardwareStateAssemblerConfig {
  double wheel_radius = 0.05;
  double thigh_length = 0.18;
  double calf_length = 0.225;
  double left_hip_offset_rad = 143.944 * kHardwareStatePi / 180.0;
  double right_hip_offset_rad = 145.56 * kHardwareStatePi / 180.0;
  double left_knee_offset_rad = 26.04 * kHardwareStatePi / 180.0;
  double right_knee_offset_rad = 33.843 * kHardwareStatePi / 180.0;
  double phi_rate_low_pass_alpha = 0.95;
};

struct HardwareStateAssemblyInput {
  std::array<double, 6> joint_position {};
  std::array<double, 6> joint_velocity {};
};

struct HardwareStateAssemblyState {
  double previous_left_phi = 0.0;
  double previous_right_phi = 0.0;
  double filtered_left_phi_rate = 0.0;
  double filtered_right_phi_rate = 0.0;
  bool has_previous_left_phi = false;
  bool has_previous_right_phi = false;
  double body_distance = 0.0;
};

struct HardwareLegKinematics {
  double hip_absolute = 0.0;
  double calf_absolute = 0.0;
  double leg_length = 0.0;
  double phi = 0.0;
  double phi_rate = 0.0;
};

struct HardwareStateAssemblyOutput {
  double body_distance = 0.0;
  double body_velocity = 0.0;
  HardwareLegKinematics left_leg;
  HardwareLegKinematics right_leg;
};

double NormalizeAngleDelta(double angle_delta);

HardwareLegKinematics ComputeHardwareLegKinematics(
    double hip_joint_position,
    double knee_joint_position,
    double hip_offset_rad,
    double knee_offset_rad,
    double previous_phi,
    double filtered_phi_rate,
    bool has_previous_phi,
    double dt,
    const HardwareStateAssemblerConfig& config);

HardwareStateAssemblyOutput AssembleHardwareState(
    const HardwareStateAssemblyInput& input,
    double dt,
    HardwareStateAssemblyState* state,
    const HardwareStateAssemblerConfig& config = HardwareStateAssemblerConfig{});

}  // namespace wheel_leg_stm32_bridge

#endif  // WHEEL_LEG_STM32_BRIDGE__HARDWARE_STATE_ASSEMBLER_HPP_
