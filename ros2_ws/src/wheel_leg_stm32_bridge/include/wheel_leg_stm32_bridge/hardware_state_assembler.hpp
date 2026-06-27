#ifndef WHEEL_LEG_STM32_BRIDGE__HARDWARE_STATE_ASSEMBLER_HPP_
#define WHEEL_LEG_STM32_BRIDGE__HARDWARE_STATE_ASSEMBLER_HPP_

#include <array>

namespace wheel_leg_stm32_bridge {

inline constexpr double kHardwareStatePi = 3.14159265358979323846;

struct HardwareStateAssemblerConfig {
  double wheel_radius = 0.05;
  double thigh_length = 0.18;
  double calf_length = 0.225;
  double phi_rate_low_pass_alpha = 0.60;
  double length_rate_low_pass_alpha = 0.62;
  double body_velocity_low_pass_alpha = 0.73;
};

struct HardwareStateAssemblyInput {
  std::array<double, 6> joint_position {};
  std::array<double, 6> joint_velocity {};
};

struct HardwareStateAssemblyState {
  double previous_left_phi = 0.0;
  double previous_right_phi = 0.0;
  double previous_left_leg_length = 0.0;
  double previous_right_leg_length = 0.0;
  double filtered_left_phi_rate = 0.0;
  double filtered_right_phi_rate = 0.0;
  double filtered_left_length_rate = 0.0;
  double filtered_right_length_rate = 0.0;
  double filtered_body_velocity = 0.0;
  bool has_previous_left_phi = false;
  bool has_previous_right_phi = false;
  bool has_previous_left_leg_length = false;
  bool has_previous_right_leg_length = false;
  bool has_previous_body_velocity = false;
  double raw_body_distance = 0.0;
  double filtered_body_distance = 0.0;
};

struct HardwareStateSignal {
  double raw = 0.0;
  double filtered = 0.0;
};

struct HardwareLegKinematics {
  double hip_absolute = 0.0;
  double calf_absolute = 0.0;
  double leg_length = 0.0;
  double phi = 0.0;
  HardwareStateSignal phi_rate;
  HardwareStateSignal length_rate;
};

struct HardwareStateAssemblyOutput {
  HardwareStateSignal body_distance;
  HardwareStateSignal body_velocity;
  HardwareLegKinematics left_leg;
  HardwareLegKinematics right_leg;
};

double NormalizeAngleDelta(double angle_delta);

HardwareLegKinematics ComputeHardwareLegKinematics(
    double hip_absolute,
    double calf_absolute,
    double previous_leg_length,
    double previous_phi,
    double filtered_length_rate,
    double filtered_phi_rate,
    bool has_previous_leg_length,
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
