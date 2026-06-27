#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "wheel_leg_bridge/message_conversions.hpp"
#include "wheel_leg_control/controller_orchestrator.hpp"
#include "wheel_leg_control/legacy_algorithms.hpp"
#include "wheel_leg_control/stand_runtime_defaults.hpp"
#include "wheel_leg_control/state_message_conversions.hpp"
#include "wheel_leg_msgs/msg/body_command.hpp"
#include "wheel_leg_msgs/msg/control_loop_debug.hpp"
#include "wheel_leg_msgs/msg/rc_status.hpp"

namespace wheel_leg_control {
namespace {

constexpr double kDefaultExpectedDtSec = 0.01;
constexpr double kDefaultAcceptedDtToleranceSec = 0.0025;
constexpr std::size_t kWarmupSamplesRequired = 3;
constexpr std::size_t kDtDebugSamplesToLog = 5;
constexpr std::size_t kDefaultTraceCapacity = 300;
constexpr const char* kDefaultTracePath = "/tmp/wheel_leg_takeover_trace.csv";

constexpr const char* kModeStand = "stand";
constexpr const char* kModeVelocity = "velocity";
constexpr const char* kModeDisabled = "disabled";
constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;

double ToSec(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

double DegreesToRadiansLocal(double degrees) {
  return degrees * kRadiansPerDegree;
}

double RadiansToDegrees(double radians) {
  return radians / kRadiansPerDegree;
}

double FindEffortForJoint(
    const wheel_leg_common::ControlCommand& command,
    const std::string& joint_name) {
  for (const auto& joint_effort : command.joint_efforts) {
    if (joint_effort.joint_name == joint_name) {
      return joint_effort.effort;
    }
  }
  return 0.0;
}

struct TraceSample {
  double state_time_sec = 0.0;
  double dt = 0.0;
  StandControlState state;
  wheel_leg_common::ControlCommand raw_command;
  wheel_leg_common::ControlCommand clamped_command;
};

enum class Mode {
  kStand,
  kVelocity,
  kDisabled,
};

struct ArbitrationResult {
  Mode mode = Mode::kStand;
  ControlTargets targets = DefaultStandControlTargets();
  bool hold_output = false;
  std::string reason;
};

struct TargetFilterState {
  double filtered_velocity = 0.0;
  double filtered_yaw_rate = 0.0;
  double filtered_leg_length = DefaultStandControlTargets().target_leg_length;
  bool initialized = false;
};

double ApplyFirstOrderLowPass(
    double current,
    double target,
    double rc,
    double dt) {
  if (rc <= 0.0 || dt <= 0.0) {
    return target;
  }
  const double alpha = std::clamp(dt / (rc + dt), 0.0, 1.0);
  return current + alpha * (target - current);
}

double ApplySlewRateLimit(
    double current,
    double target,
    double max_rate,
    double dt) {
  if (max_rate <= 0.0 || dt <= 0.0) {
    return target;
  }
  const double delta_limit = max_rate * dt;
  return current + std::clamp(target - current, -delta_limit, delta_limit);
}

std::string ModeToString(Mode mode) {
  switch (mode) {
    case Mode::kStand:
      return kModeStand;
    case Mode::kVelocity:
      return kModeVelocity;
    case Mode::kDisabled:
      return kModeDisabled;
  }
  return kModeStand;
}

Mode ModeFromString(const std::string& mode_text) {
  if (mode_text == kModeVelocity) {
    return Mode::kVelocity;
  }
  if (mode_text == kModeDisabled) {
    return Mode::kDisabled;
  }
  return Mode::kStand;
}

bool IsOperationalModeReason(const std::string& reason) {
  return reason == "mode_stand" || reason == "mode_velocity" ||
         reason == "mode_disabled";
}

wheel_leg_common::ControlCommand BuildZeroCommand(double state_time_sec) {
  wheel_leg_common::ControlCommand command;
  command.stamp.sec = static_cast<std::int32_t>(state_time_sec);
  command.stamp.nanosec = static_cast<std::uint32_t>(
      (state_time_sec - static_cast<double>(command.stamp.sec)) *
      1000000000.0);
  command.joint_efforts = {
      {"right_hip", 0.0},
      {"right_knee", 0.0},
      {"left_hip", 0.0},
      {"left_knee", 0.0},
      {"right_wheel", 0.0},
      {"left_wheel", 0.0},
  };
  return command;
}

}  // namespace

class ControllerNode : public rclcpp::Node {
 public:
  ControllerNode()
      : rclcpp::Node("wheel_leg_controller"),
        default_targets_(DefaultStandControlTargets()) {
    pid_defaults_.leg_length = LoadPidConfig(
        "leg_length_pid", DefaultStandLegacyPidDefaults().leg_length);
    pid_defaults_.steer_velocity = LoadPidConfig(
        "steer_pid", DefaultStandLegacyPidDefaults().steer_velocity);
    pid_defaults_.anti_crash = LoadPidConfig(
        "anti_crash_pid", DefaultStandLegacyPidDefaults().anti_crash);
    pid_defaults_.roll_balance = LoadPidConfig(
        "roll_balance_pid", DefaultStandLegacyPidDefaults().roll_balance);

    publish_control_command_ =
        declare_parameter<bool>("publish_control_command", true);
    stage_config_.enable_vmc =
        declare_parameter<bool>("enable_vmc", stage_config_.enable_vmc);
    stage_config_.enable_lqr =
        declare_parameter<bool>("enable_lqr", stage_config_.enable_lqr);
    stage_config_.enable_leg_length_pid = declare_parameter<bool>(
        "enable_leg_length_pid", stage_config_.enable_leg_length_pid);
    stage_config_.enable_heading_control = declare_parameter<bool>(
        "enable_heading_control", stage_config_.enable_heading_control);
    stage_config_.enable_anti_split = declare_parameter<bool>(
        "enable_anti_split", stage_config_.enable_anti_split);
    stage_config_.enable_roll_compensation = declare_parameter<bool>(
        "enable_roll_compensation", stage_config_.enable_roll_compensation);
    stage_config_.enable_wheel_output = declare_parameter<bool>(
        "enable_wheel_output", stage_config_.enable_wheel_output);
    stage_config_.enable_hip_output = declare_parameter<bool>(
        "enable_hip_output", stage_config_.enable_hip_output);
    stage_config_.enable_knee_output = declare_parameter<bool>(
        "enable_knee_output", stage_config_.enable_knee_output);
    hip_effort_limit_ =
        declare_parameter<double>("hip_effort_limit", 50.0);
    knee_effort_limit_ =
        declare_parameter<double>("knee_effort_limit", 50.0);
    wheel_effort_limit_ =
        declare_parameter<double>("wheel_effort_limit", 20.0);
    trace_capture_enabled_ =
        declare_parameter<bool>("trace_capture_enabled", true);
    trace_capacity_ =
        declare_parameter<int>("trace_capacity", kDefaultTraceCapacity);
    trace_output_path_ =
        declare_parameter<std::string>("trace_output_path", kDefaultTracePath);
    command_timeout_sec_ =
        declare_parameter<double>("command_timeout_sec", 0.2);
    target_velocity_scale_ =
        declare_parameter<double>("target_velocity_scale", 0.6);
    target_yaw_rate_scale_ =
        declare_parameter<double>("target_yaw_rate_scale", 2.0);
    yaw_rate_assist_scale_ =
        declare_parameter<double>("yaw_rate_assist_scale", 0.0);
    turn_hip_feedforward_scale_ = declare_parameter<double>(
        "turn_hip_feedforward_scale", DefaultTurnHipFeedforwardScale());
    body_height_offset_scale_ =
        declare_parameter<double>("body_height_offset_scale", 0.2);
    target_leg_length_min_ =
        declare_parameter<double>("target_leg_length_min", 0.23);
    target_leg_length_max_ =
        declare_parameter<double>("target_leg_length_max", 0.33);
    target_phi_deg_ =
        declare_parameter<double>("target_phi_deg", 97.1);
    target_phi_min_deg_ =
        declare_parameter<double>("target_phi_min_deg", 30.0);
    target_phi_max_deg_ =
        declare_parameter<double>("target_phi_max_deg", 150.0);
    target_pitch_deg_ =
        declare_parameter<double>("target_pitch_deg", 0.0);
    velocity_ref_lpf_rc_ =
        declare_parameter<double>("velocity_ref_lpf_rc", 0.12);
    velocity_ref_slew_rate_ =
        declare_parameter<double>("velocity_ref_slew_rate", 1.5);
    velocity_snap_threshold_ =
        declare_parameter<double>("velocity_snap_threshold", 0.1);
    velocity_zero_hold_threshold_ =
        declare_parameter<double>("velocity_zero_hold_threshold", 0.1);
    velocity_zero_hold_exit_threshold_ =
        declare_parameter<double>("velocity_zero_hold_exit_threshold", 0.15);
    velocity_zero_hold_entry_delay_sec_ =
        declare_parameter<double>("velocity_zero_hold_entry_delay_sec", 0.08);
    zero_hold_measurement_clamp_threshold_ =
        declare_parameter<double>("zero_hold_measurement_clamp_threshold", 0.1);
    zero_hold_wheel_effort_limit_ =
        declare_parameter<double>("zero_hold_wheel_effort_limit", -1.0);
    yaw_rate_ref_lpf_rc_ =
        declare_parameter<double>("yaw_rate_ref_lpf_rc", 0.4);
    yaw_rate_ref_slew_rate_ =
        declare_parameter<double>("yaw_rate_ref_slew_rate", 1.8);
    leg_length_ref_lpf_rc_ =
        declare_parameter<double>("leg_length_ref_lpf_rc", 0.15);
    debug_plot_publish_hz_ =
        declare_parameter<double>("debug_plot_publish_hz", 100.0);
    disabled_stops_publishing_ =
        declare_parameter<bool>("disabled_stops_publishing", true);
    expected_dt_sec_ =
        declare_parameter<double>("expected_dt_sec", kDefaultExpectedDtSec);
    accepted_dt_tolerance_sec_ = declare_parameter<double>(
        "accepted_dt_tolerance_sec", kDefaultAcceptedDtToleranceSec);
    declare_parameter<double>("publish_rate_hz", 500.0);
    RCLCPP_WARN(
        get_logger(),
        "Parameter publish_rate_hz is ignored; /wheel_leg_controller now runs one control step per new /robot_state sample.");

    target_phi_deg_ =
        std::clamp(target_phi_deg_, target_phi_min_deg_, target_phi_max_deg_);
    default_targets_.target_phi = DegreesToRadiansLocal(target_phi_deg_);
    default_targets_.target_pitch = DegreesToRadiansLocal(target_pitch_deg_);
    latest_control_mode_ = kModeStand;
    orchestrator_.ConfigurePidDefaults(pid_defaults_);
    orchestrator_.SetTurnHipFeedforwardScale(turn_hip_feedforward_scale_);
    orchestrator_.SetStageConfig(stage_config_);
    parameter_callback_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
          return HandleParameterUpdate(parameters);
        });

    control_state_sub_ =
        create_subscription<wheel_leg_msgs::msg::StandControlState>(
            "/robot_state", rclcpp::SystemDefaultsQoS(),
            [this](const wheel_leg_msgs::msg::StandControlState::SharedPtr msg) {
              OnControlState(*msg);
            });
    cmd_vel_sub_ =
        create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
              latest_cmd_vel_ = *msg;
              last_cmd_vel_time_sec_ = now().seconds();
            });
    control_mode_sub_ =
        create_subscription<std_msgs::msg::String>(
            "/control_mode", rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::String::SharedPtr msg) {
              latest_control_mode_ = msg->data;
              last_control_mode_time_sec_ = now().seconds();
            });
    body_cmd_sub_ =
        create_subscription<wheel_leg_msgs::msg::BodyCommand>(
            "/body_cmd", rclcpp::SystemDefaultsQoS(),
            [this](const wheel_leg_msgs::msg::BodyCommand::SharedPtr msg) {
              latest_body_cmd_ = *msg;
              last_body_cmd_time_sec_ = now().seconds();
            });
    rc_status_sub_ =
        create_subscription<wheel_leg_msgs::msg::RcStatus>(
            "/rc/status", rclcpp::SystemDefaultsQoS(),
            [this](const wheel_leg_msgs::msg::RcStatus::SharedPtr msg) {
              latest_rc_status_ = *msg;
              last_rc_status_time_sec_ = now().seconds();
            });
    joint_command_pub_ =
        create_publisher<wheel_leg_msgs::msg::JointCommand>(
            "/joint_command", rclcpp::SystemDefaultsQoS());
    velocity_debug_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/control/velocity", rclcpp::SystemDefaultsQoS());
    yaw_debug_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/control/yaw", rclcpp::SystemDefaultsQoS());
    leg_length_debug_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/control/leg_length", rclcpp::SystemDefaultsQoS());
    leg_length_output_debug_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/control/leg_length_output", rclcpp::SystemDefaultsQoS());
    anti_crash_debug_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/control/anti_crash", rclcpp::SystemDefaultsQoS());
    roll_balance_debug_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/control/roll_balance", rclcpp::SystemDefaultsQoS());
    velocity_plot_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/plot/velocity", rclcpp::SystemDefaultsQoS());
    yaw_plot_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/plot/yaw", rclcpp::SystemDefaultsQoS());
    leg_length_plot_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/plot/leg_length", rclcpp::SystemDefaultsQoS());
    leg_length_output_plot_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/plot/leg_length_output", rclcpp::SystemDefaultsQoS());
    anti_crash_plot_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/plot/anti_crash", rclcpp::SystemDefaultsQoS());
    roll_balance_plot_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/plot/roll_balance", rclcpp::SystemDefaultsQoS());
    wheel_effort_debug_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/control/wheel_effort", rclcpp::SystemDefaultsQoS());
    wheel_effort_plot_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/plot/wheel_effort", rclcpp::SystemDefaultsQoS());
    balance_debug_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/control/balance", rclcpp::SystemDefaultsQoS());
    balance_plot_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/plot/balance", rclcpp::SystemDefaultsQoS());
    turn_internal_debug_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/control/turn_internal", rclcpp::SystemDefaultsQoS());
    turn_internal_plot_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/plot/turn_internal", rclcpp::SystemDefaultsQoS());
    velocity_ref_filter_plot_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/plot/ref_filter/velocity", rclcpp::SystemDefaultsQoS());
    yaw_rate_ref_filter_plot_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/plot/ref_filter/yaw_rate", rclcpp::SystemDefaultsQoS());
    leg_length_ref_filter_plot_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/debug/plot/ref_filter/leg_length", rclcpp::SystemDefaultsQoS());
  }

 private:
  rcl_interfaces::msg::SetParametersResult HandleParameterUpdate(
      const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    double next_target_phi_deg = target_phi_deg_;
    double next_target_phi_min_deg = target_phi_min_deg_;
    double next_target_phi_max_deg = target_phi_max_deg_;
    double next_target_pitch_deg = target_pitch_deg_;
    double next_velocity_ref_lpf_rc = velocity_ref_lpf_rc_;
    double next_velocity_ref_slew_rate = velocity_ref_slew_rate_;
    double next_velocity_snap_threshold = velocity_snap_threshold_;
    double next_yaw_rate_ref_lpf_rc = yaw_rate_ref_lpf_rc_;
    double next_yaw_rate_ref_slew_rate = yaw_rate_ref_slew_rate_;
    double next_leg_length_ref_lpf_rc = leg_length_ref_lpf_rc_;
    double next_yaw_rate_assist_scale = yaw_rate_assist_scale_;
    double next_turn_hip_feedforward_scale = turn_hip_feedforward_scale_;
    double next_expected_dt_sec = expected_dt_sec_;
    double next_accepted_dt_tolerance_sec = accepted_dt_tolerance_sec_;
    StandControlStageConfig next_stage_config = stage_config_;

    for (const auto& parameter : parameters) {
      if (parameter.get_name() == "enable_vmc") {
        next_stage_config.enable_vmc = parameter.as_bool();
        continue;
      }

      if (parameter.get_name() == "enable_lqr") {
        next_stage_config.enable_lqr = parameter.as_bool();
        continue;
      }

      if (parameter.get_name() == "enable_leg_length_pid") {
        next_stage_config.enable_leg_length_pid = parameter.as_bool();
        continue;
      }

      if (parameter.get_name() == "enable_heading_control") {
        next_stage_config.enable_heading_control = parameter.as_bool();
        continue;
      }

      if (parameter.get_name() == "enable_anti_split") {
        next_stage_config.enable_anti_split = parameter.as_bool();
        continue;
      }

      if (parameter.get_name() == "enable_roll_compensation") {
        next_stage_config.enable_roll_compensation = parameter.as_bool();
        continue;
      }

      if (parameter.get_name() == "enable_wheel_output") {
        next_stage_config.enable_wheel_output = parameter.as_bool();
        continue;
      }

      if (parameter.get_name() == "enable_hip_output") {
        next_stage_config.enable_hip_output = parameter.as_bool();
        continue;
      }

      if (parameter.get_name() == "enable_knee_output") {
        next_stage_config.enable_knee_output = parameter.as_bool();
        continue;
      }

      if (parameter.get_name() == "target_phi_deg") {
        next_target_phi_deg = parameter.as_double();
        if (!std::isfinite(next_target_phi_deg)) {
          result.successful = false;
          result.reason = "target_phi_deg must be finite";
          return result;
        }
        continue;
      }

      if (parameter.get_name() == "target_phi_min_deg") {
        next_target_phi_min_deg = parameter.as_double();
        if (!std::isfinite(next_target_phi_min_deg)) {
          result.successful = false;
          result.reason = "target_phi_min_deg must be finite";
          return result;
        }
        continue;
      }

      if (parameter.get_name() == "target_phi_max_deg") {
        next_target_phi_max_deg = parameter.as_double();
        if (!std::isfinite(next_target_phi_max_deg)) {
          result.successful = false;
          result.reason = "target_phi_max_deg must be finite";
          return result;
        }
        continue;
      }

      if (parameter.get_name() == "target_pitch_deg") {
        next_target_pitch_deg = parameter.as_double();
        if (!std::isfinite(next_target_pitch_deg)) {
          result.successful = false;
          result.reason = "target_pitch_deg must be finite";
          return result;
        }
        continue;
      }

      if (parameter.get_name() == "expected_dt_sec") {
        next_expected_dt_sec = parameter.as_double();
        if (!std::isfinite(next_expected_dt_sec) || next_expected_dt_sec <= 0.0) {
          result.successful = false;
          result.reason = "expected_dt_sec must be finite and > 0";
          return result;
        }
        continue;
      }

      if (parameter.get_name() == "accepted_dt_tolerance_sec") {
        next_accepted_dt_tolerance_sec = parameter.as_double();
        if (!IsFiniteAndNonNegative(next_accepted_dt_tolerance_sec)) {
          result.successful = false;
          result.reason =
              "accepted_dt_tolerance_sec must be finite and non-negative";
          return result;
        }
        continue;
      }

      if (parameter.get_name() == "velocity_ref_lpf_rc") {
        next_velocity_ref_lpf_rc = parameter.as_double();
        if (!IsFiniteAndNonNegative(next_velocity_ref_lpf_rc)) {
          result.successful = false;
          result.reason = "velocity_ref_lpf_rc must be finite and non-negative";
          return result;
        }
        continue;
      }

      if (parameter.get_name() == "velocity_ref_slew_rate") {
        next_velocity_ref_slew_rate = parameter.as_double();
        if (!IsFiniteAndNonNegative(next_velocity_ref_slew_rate)) {
          result.successful = false;
          result.reason =
              "velocity_ref_slew_rate must be finite and non-negative";
          return result;
        }
        continue;
      }

      if (parameter.get_name() == "velocity_snap_threshold") {
        next_velocity_snap_threshold = parameter.as_double();
        if (!IsFiniteAndNonNegative(next_velocity_snap_threshold)) {
          result.successful = false;
          result.reason =
              "velocity_snap_threshold must be finite and non-negative";
          return result;
        }
        continue;
      }

      if (parameter.get_name() == "yaw_rate_ref_lpf_rc") {
        next_yaw_rate_ref_lpf_rc = parameter.as_double();
        if (!IsFiniteAndNonNegative(next_yaw_rate_ref_lpf_rc)) {
          result.successful = false;
          result.reason = "yaw_rate_ref_lpf_rc must be finite and non-negative";
          return result;
        }
        continue;
      }

      if (parameter.get_name() == "yaw_rate_ref_slew_rate") {
        next_yaw_rate_ref_slew_rate = parameter.as_double();
        if (!IsFiniteAndNonNegative(next_yaw_rate_ref_slew_rate)) {
          result.successful = false;
          result.reason =
              "yaw_rate_ref_slew_rate must be finite and non-negative";
          return result;
        }
        continue;
      }

      if (parameter.get_name() == "leg_length_ref_lpf_rc") {
        next_leg_length_ref_lpf_rc = parameter.as_double();
        if (!IsFiniteAndNonNegative(next_leg_length_ref_lpf_rc)) {
          result.successful = false;
          result.reason =
              "leg_length_ref_lpf_rc must be finite and non-negative";
          return result;
        }
        continue;
      }

      if (parameter.get_name() == "yaw_rate_assist_scale") {
        next_yaw_rate_assist_scale = parameter.as_double();
        if (!std::isfinite(next_yaw_rate_assist_scale)) {
          result.successful = false;
          result.reason = "yaw_rate_assist_scale must be finite";
          return result;
        }
        continue;
      }

      if (parameter.get_name() == "turn_hip_feedforward_scale") {
        next_turn_hip_feedforward_scale = parameter.as_double();
        if (!std::isfinite(next_turn_hip_feedforward_scale)) {
          result.successful = false;
          result.reason = "turn_hip_feedforward_scale must be finite";
          return result;
        }
        continue;
      }
    }

    if (next_target_phi_min_deg > next_target_phi_max_deg) {
      result.successful = false;
      result.reason = "target_phi_min_deg must be <= target_phi_max_deg";
      return result;
    }

    target_phi_min_deg_ = next_target_phi_min_deg;
    target_phi_max_deg_ = next_target_phi_max_deg;
    target_phi_deg_ = std::clamp(
        next_target_phi_deg, target_phi_min_deg_, target_phi_max_deg_);
    target_pitch_deg_ = next_target_pitch_deg;
    default_targets_.target_phi = DegreesToRadiansLocal(target_phi_deg_);
    default_targets_.target_pitch = DegreesToRadiansLocal(target_pitch_deg_);
    velocity_ref_lpf_rc_ = next_velocity_ref_lpf_rc;
    velocity_ref_slew_rate_ = next_velocity_ref_slew_rate;
    velocity_snap_threshold_ = next_velocity_snap_threshold;
    yaw_rate_ref_lpf_rc_ = next_yaw_rate_ref_lpf_rc;
    yaw_rate_ref_slew_rate_ = next_yaw_rate_ref_slew_rate;
    leg_length_ref_lpf_rc_ = next_leg_length_ref_lpf_rc;
    yaw_rate_assist_scale_ = next_yaw_rate_assist_scale;
    turn_hip_feedforward_scale_ = next_turn_hip_feedforward_scale;
    expected_dt_sec_ = next_expected_dt_sec;
    accepted_dt_tolerance_sec_ = next_accepted_dt_tolerance_sec;
    const bool stage_config_changed =
        next_stage_config.enable_vmc != stage_config_.enable_vmc ||
        next_stage_config.enable_lqr != stage_config_.enable_lqr ||
        next_stage_config.enable_leg_length_pid !=
            stage_config_.enable_leg_length_pid ||
        next_stage_config.enable_heading_control !=
            stage_config_.enable_heading_control ||
        next_stage_config.enable_anti_split !=
            stage_config_.enable_anti_split ||
        next_stage_config.enable_roll_compensation !=
            stage_config_.enable_roll_compensation ||
        next_stage_config.enable_wheel_output !=
            stage_config_.enable_wheel_output ||
        next_stage_config.enable_hip_output != stage_config_.enable_hip_output ||
        next_stage_config.enable_knee_output != stage_config_.enable_knee_output;
    stage_config_ = next_stage_config;
    orchestrator_.SetTurnHipFeedforwardScale(turn_hip_feedforward_scale_);
    orchestrator_.SetStageConfig(stage_config_);
    if (stage_config_changed) {
      RCLCPP_WARN(
          get_logger(),
          "Updated control stage gates: vmc=%s lqr=%s leg_length=%s heading=%s anti_split=%s roll=%s wheel=%s hip=%s knee=%s",
          stage_config_.enable_vmc ? "true" : "false",
          stage_config_.enable_lqr ? "true" : "false",
          stage_config_.enable_leg_length_pid ? "true" : "false",
          stage_config_.enable_heading_control ? "true" : "false",
          stage_config_.enable_anti_split ? "true" : "false",
          stage_config_.enable_roll_compensation ? "true" : "false",
          stage_config_.enable_wheel_output ? "true" : "false",
          stage_config_.enable_hip_output ? "true" : "false",
          stage_config_.enable_knee_output ? "true" : "false");
    }
    invalid_dt_warning_logged_ = false;
    out_of_range_dt_warning_logged_ = false;
    return result;
  }

  bool IsFiniteAndNonNegative(double value) const {
    return std::isfinite(value) && value >= 0.0;
  }

  double MinAcceptedDtSec() const {
    return std::max(0.0, expected_dt_sec_ - accepted_dt_tolerance_sec_);
  }

  double MaxAcceptedDtSec() const {
    return expected_dt_sec_ + accepted_dt_tolerance_sec_;
  }

  LegacyPidConfig LoadPidConfig(
      const std::string& prefix,
      const LegacyPidConfig& defaults) {
    LegacyPidConfig config = defaults;
    config.kp = declare_parameter<double>(prefix + ".kp", defaults.kp);
    config.ki = declare_parameter<double>(prefix + ".ki", defaults.ki);
    config.kd = declare_parameter<double>(prefix + ".kd", defaults.kd);
    config.max_output =
        declare_parameter<double>(prefix + ".max_output", defaults.max_output);
    config.integral_limit = declare_parameter<double>(
        prefix + ".integral_limit", defaults.integral_limit);
    config.deadband =
        declare_parameter<double>(prefix + ".deadband", defaults.deadband);
    config.improvement_flags = static_cast<std::uint32_t>(
        declare_parameter<int>(prefix + ".improvement_flags",
                               static_cast<int>(defaults.improvement_flags)));
    config.coef_a =
        declare_parameter<double>(prefix + ".coef_a", defaults.coef_a);
    config.coef_b =
        declare_parameter<double>(prefix + ".coef_b", defaults.coef_b);
    config.output_lpf_rc = declare_parameter<double>(
        prefix + ".output_lpf_rc", defaults.output_lpf_rc);
    config.derivative_lpf_rc = declare_parameter<double>(
        prefix + ".derivative_lpf_rc", defaults.derivative_lpf_rc);
    return config;
  }

  void OnControlState(const wheel_leg_msgs::msg::StandControlState& msg) {
    const double state_time_sec = ToSec(msg.header.stamp);
    if (!std::isfinite(state_time_sec)) {
      return;
    }

    if (has_processed_state_ && state_time_sec <= last_processed_state_time_sec_) {
      if (!non_increasing_stamp_warning_logged_) {
        RCLCPP_WARN(
            get_logger(),
            "Ignoring /robot_state sample with non-increasing stamp: current=%.9f, last=%.9f",
            state_time_sec,
            last_processed_state_time_sec_);
        non_increasing_stamp_warning_logged_ = true;
      }
      return;
    }

    const StandControlState control_state =
        wheel_leg_control::FromRosStandControlState(msg);
    if (!has_processed_state_) {
      last_processed_state_time_sec_ = state_time_sec;
      has_processed_state_ = true;
      warmup_sample_count_ = 1;
      return;
    }

    const double raw_dt = state_time_sec - last_processed_state_time_sec_;
    last_processed_state_time_sec_ = state_time_sec;

    if (!std::isfinite(raw_dt) || raw_dt <= 0.0) {
      if (!invalid_dt_warning_logged_) {
        RCLCPP_WARN(
            get_logger(),
            "Ignoring /robot_state sample with invalid dt=%.9f",
            raw_dt);
        invalid_dt_warning_logged_ = true;
      }
      return;
    }

    const double min_accepted_dt_sec = MinAcceptedDtSec();
    const double max_accepted_dt_sec = MaxAcceptedDtSec();
    if (raw_dt < min_accepted_dt_sec || raw_dt > max_accepted_dt_sec) {
      if (!out_of_range_dt_warning_logged_) {
        RCLCPP_WARN(
            get_logger(),
            "Ignoring /robot_state sample with out-of-range dt=%.9f; expected %.9f s within [%.9f, %.9f]",
            raw_dt, expected_dt_sec_, min_accepted_dt_sec, max_accepted_dt_sec);
        out_of_range_dt_warning_logged_ = true;
      }
      return;
    }

    const double dt = std::clamp(raw_dt, min_accepted_dt_sec, max_accepted_dt_sec);
    if (dt_debug_log_count_ < kDtDebugSamplesToLog) {
      ++dt_debug_log_count_;
      RCLCPP_INFO(
          get_logger(),
          "Controller dt sample %zu: %.9f s",
          dt_debug_log_count_,
          dt);
    }

    if (warmup_sample_count_ < kWarmupSamplesRequired) {
      ++warmup_sample_count_;
      if (warmup_sample_count_ == kWarmupSamplesRequired) {
        orchestrator_.SetTargets(default_targets_);
        orchestrator_.ResetControllersForState(control_state);
      }
      return;
    }

    const ArbitrationResult arbitration =
        BuildArbitrationResult(now().seconds());
    ApplyArbitration(arbitration, control_state, state_time_sec, dt);

    if (!publish_control_command_) {
      PublishWheelEffortDebug(BuildZeroCommand(state_time_sec), state_time_sec);
      return;
    }

    if (last_effective_mode_.has_value() &&
        *last_effective_mode_ == Mode::kDisabled &&
        !disabled_stops_publishing_) {
      const auto zero_command = BuildZeroCommand(state_time_sec);
      PublishWheelEffortDebug(zero_command, state_time_sec);
      joint_command_pub_->publish(
          wheel_leg_bridge::ToRosJointCommand(zero_command));
      return;
    }

    if (arbitration.hold_output) {
      PublishWheelEffortDebug(BuildZeroCommand(state_time_sec), state_time_sec);
      return;
    }

    const StandControlState effective_control_state =
        BuildEffectiveControlState(control_state);
    const auto step_outputs =
        orchestrator_.Step(state_time_sec, dt, effective_control_state);
    if (!step_outputs.has_value()) {
      PublishWheelEffortDebug(BuildZeroCommand(state_time_sec), state_time_sec);
      return;
    }

    auto limited_command = step_outputs->command;
    ClampCommand(&limited_command);
    ApplyZeroHoldCommandLimit(&limited_command);
    PublishWheelEffortDebug(limited_command, state_time_sec);
    PublishLegLengthOutputDebug(*step_outputs, state_time_sec);
    PublishRollBalanceDebug(*step_outputs, control_state, state_time_sec);
    PublishTurnInternalDebug(*step_outputs, state_time_sec);
    RecordTraceSample(
        state_time_sec, dt, control_state, step_outputs->command, limited_command);
    joint_command_pub_->publish(
        wheel_leg_bridge::ToRosJointCommand(limited_command));
  }

  ArbitrationResult BuildArbitrationResult(double now_sec) const {
    ArbitrationResult result;
    result.mode = ModeFromString(latest_control_mode_);
    result.targets = default_targets_;

    if (IsCommandTimedOut(last_control_mode_time_sec_, now_sec)) {
      result.reason = "control_mode_timeout";
      result.mode = Mode::kStand;
      return result;
    }
    if (IsCommandTimedOut(last_cmd_vel_time_sec_, now_sec)) {
      result.reason = "cmd_vel_timeout";
      result.mode = Mode::kStand;
      return result;
    }
    if (IsCommandTimedOut(last_body_cmd_time_sec_, now_sec)) {
      result.reason = "body_cmd_timeout";
      result.mode = Mode::kStand;
      return result;
    }
    if (!latest_rc_status_.has_value()) {
      result.reason = "rc_status_missing";
      result.mode = Mode::kStand;
      return result;
    }
    if (IsCommandTimedOut(last_rc_status_time_sec_, now_sec)) {
      result.reason = "rc_status_timeout";
      result.mode = Mode::kStand;
      return result;
    }
    if (!latest_rc_status_->serial_online) {
      result.reason = "rc_serial_offline";
      result.mode = Mode::kStand;
      return result;
    }
    if (latest_rc_status_->frame_timeout) {
      result.reason = "rc_frame_timeout";
      result.mode = Mode::kStand;
      return result;
    }
    if (latest_rc_status_->failsafe) {
      result.reason = "rc_failsafe";
      result.mode = Mode::kStand;
      return result;
    }

    result.targets.target_leg_length = std::clamp(
        default_targets_.target_leg_length +
            latest_body_cmd_.body_height_offset * body_height_offset_scale_,
        target_leg_length_min_, target_leg_length_max_);

    switch (result.mode) {
      case Mode::kStand:
        result.reason = "mode_stand";
        break;
      case Mode::kVelocity:
        result.targets.target_velocity =
            latest_cmd_vel_.linear.x * target_velocity_scale_;
        result.targets.target_yaw_rate =
            latest_cmd_vel_.angular.z * target_yaw_rate_scale_ +
            latest_body_cmd_.yaw_rate_assist * yaw_rate_assist_scale_;
        result.reason = "mode_velocity";
        break;
      case Mode::kDisabled:
        result.reason = "mode_disabled";
        result.hold_output = disabled_stops_publishing_;
        break;
    }

    return result;
  }

  bool IsCommandTimedOut(double last_update_sec, double now_sec) const {
    if (last_update_sec < 0.0) {
      return true;
    }
    return now_sec - last_update_sec > command_timeout_sec_;
  }

  StandControlState BuildEffectiveControlState(
      const StandControlState& control_state) const {
    StandControlState effective_state = control_state;
    if (zero_hold_active_ &&
        std::abs(effective_state.body.velocity) <=
            zero_hold_measurement_clamp_threshold_) {
      effective_state.body.velocity = 0.0;
    }
    return effective_state;
  }

  void ApplyArbitration(const ArbitrationResult& arbitration,
                        const StandControlState& control_state,
                        double state_time_sec,
                        double dt) {
    const bool mode_changed = !last_effective_mode_.has_value() ||
                              arbitration.mode != *last_effective_mode_;
    const bool reason_changed =
        !last_arbitration_reason_.has_value() ||
        arbitration.reason != *last_arbitration_reason_;
    const bool fallback_active = !IsOperationalModeReason(arbitration.reason);
    const bool fallback_changed = fallback_active_ != fallback_active;
    ControlTargets filtered_targets =
        ApplyTargetFiltering(arbitration, control_state, state_time_sec, dt,
                             mode_changed, fallback_active);
    PublishDebugTopics(
        arbitration.targets, filtered_targets, control_state, state_time_sec);
    orchestrator_.SetTargets(filtered_targets);

    if (mode_changed || reason_changed) {
      orchestrator_.ResetControllersForState(control_state);
    }

    if (fallback_changed || (fallback_active && reason_changed)) {
      if (fallback_active) {
        RCLCPP_WARN(
            get_logger(),
            "Controller fallback active at %.3f s: mode=%s reason=%s",
            state_time_sec, ModeToString(arbitration.mode).c_str(),
            arbitration.reason.c_str());
      } else {
        RCLCPP_INFO(
            get_logger(),
            "Controller fallback cleared at %.3f s; mode=%s reason=%s",
            state_time_sec, ModeToString(arbitration.mode).c_str(),
            arbitration.reason.c_str());
      }
    }

    if (mode_changed) {
      RCLCPP_INFO(
          get_logger(),
          "Controller mode transition at %.3f s: %s -> %s (%s)",
          state_time_sec,
          last_effective_mode_.has_value()
              ? ModeToString(*last_effective_mode_).c_str()
              : "unset",
          ModeToString(arbitration.mode).c_str(), arbitration.reason.c_str());
    }

    fallback_active_ = fallback_active;
    last_effective_mode_ = arbitration.mode;
    last_arbitration_reason_ = arbitration.reason;
  }

  ControlTargets ApplyTargetFiltering(const ArbitrationResult& arbitration,
                                      const StandControlState& control_state,
                                      double state_time_sec,
                                      double dt,
                                      bool mode_changed,
                                      bool fallback_active) {
    ControlTargets filtered_targets = arbitration.targets;
    const bool entering_velocity =
        arbitration.mode == Mode::kVelocity &&
        (!last_effective_mode_.has_value() ||
         *last_effective_mode_ != Mode::kVelocity);
    const bool reset_to_safe_targets =
        fallback_active || arbitration.mode != Mode::kVelocity;
    zero_hold_active_ = false;
    (void)state_time_sec;

    if (!filter_state_.initialized || reset_to_safe_targets) {
      ResetZeroHoldLatch();
      return BuildSafeTargets(filtered_targets, control_state);
    }

    if (mode_changed && entering_velocity) {
      ResetZeroHoldLatch();
      velocity_mode_distance_ref_ = control_state.body.distance;
      filter_state_.filtered_velocity = 0.0;
      filter_state_.filtered_yaw_rate = 0.0;
      filter_state_.filtered_leg_length = filtered_targets.target_leg_length;
      filter_state_.initialized = true;
    }

    const double commanded_velocity = filtered_targets.target_velocity;
    const double velocity_error =
        std::abs(filter_state_.filtered_velocity - commanded_velocity);
    if (velocity_error <= velocity_snap_threshold_) {
      filter_state_.filtered_velocity = commanded_velocity;
    } else {
      const double velocity_candidate = ApplyFirstOrderLowPass(
          filter_state_.filtered_velocity,
          commanded_velocity,
          velocity_ref_lpf_rc_,
          dt);
      filter_state_.filtered_velocity = ApplySlewRateLimit(
          filter_state_.filtered_velocity,
          velocity_candidate,
          velocity_ref_slew_rate_,
          dt);
    }

    const double yaw_rate_candidate = ApplyFirstOrderLowPass(
        filter_state_.filtered_yaw_rate,
        filtered_targets.target_yaw_rate,
        yaw_rate_ref_lpf_rc_,
        dt);
    filter_state_.filtered_yaw_rate = ApplySlewRateLimit(
        filter_state_.filtered_yaw_rate,
        yaw_rate_candidate,
        yaw_rate_ref_slew_rate_,
        dt);

    filter_state_.filtered_leg_length = ApplyFirstOrderLowPass(
        filter_state_.filtered_leg_length,
        filtered_targets.target_leg_length,
        leg_length_ref_lpf_rc_,
        dt);

    velocity_mode_distance_ref_ += filter_state_.filtered_velocity * dt;
    filtered_targets.target_velocity = filter_state_.filtered_velocity;
    filtered_targets.target_yaw_rate = filter_state_.filtered_yaw_rate;
    filtered_targets.target_leg_length = filter_state_.filtered_leg_length;
    filtered_targets.target_distance = velocity_mode_distance_ref_;
    return filtered_targets;
  }

  ControlTargets BuildSafeTargets(const ControlTargets& targets,
                                  const StandControlState& control_state) {
    ResetTargetFilterState(targets);
    velocity_mode_distance_ref_ = control_state.body.distance;

    ControlTargets safe_targets = targets;
    safe_targets.target_distance = control_state.body.distance;
    safe_targets.target_velocity = 0.0;
    safe_targets.target_yaw_rate = 0.0;
    safe_targets.target_leg_length = filter_state_.filtered_leg_length;
    return safe_targets;
  }

  void ResetTargetFilterState(const ControlTargets& targets) {
    filter_state_.filtered_velocity = 0.0;
    filter_state_.filtered_yaw_rate = 0.0;
    filter_state_.filtered_leg_length = targets.target_leg_length;
    filter_state_.initialized = true;
  }

  bool UpdateZeroHoldLatch(double commanded_velocity, double state_time_sec) {
    const double command_magnitude = std::abs(commanded_velocity);
    if (command_magnitude >= velocity_zero_hold_exit_threshold_) {
      ResetZeroHoldLatch();
      return false;
    }

    if (command_magnitude <= velocity_zero_hold_threshold_) {
      if (zero_hold_candidate_start_time_sec_ < 0.0) {
        zero_hold_candidate_start_time_sec_ = state_time_sec;
      }
      if (state_time_sec - zero_hold_candidate_start_time_sec_ >=
          velocity_zero_hold_entry_delay_sec_) {
        zero_hold_latched_ = true;
      }
      return zero_hold_latched_;
    }

    return zero_hold_latched_;
  }

  void ResetZeroHoldLatch() {
    zero_hold_latched_ = false;
    zero_hold_distance_ref_initialized_ = false;
    zero_hold_distance_ref_ = 0.0;
    zero_hold_candidate_start_time_sec_ = -1.0;
  }

  void ApplyZeroHoldCommandLimit(wheel_leg_common::ControlCommand* command) {
    if (!command || !zero_hold_active_ || zero_hold_wheel_effort_limit_ < 0.0) {
      return;
    }

    for (auto& joint_effort : command->joint_efforts) {
      if (joint_effort.joint_name.find("wheel") == std::string::npos) {
        continue;
      }
      joint_effort.effort = std::clamp(
          joint_effort.effort,
          -zero_hold_wheel_effort_limit_,
          zero_hold_wheel_effort_limit_);
    }
  }

  void PublishDebugTopics(const ControlTargets& raw_targets,
                          const ControlTargets& targets,
                          const StandControlState& control_state,
                          double state_time_sec) {
    const auto stamp = [state_time_sec]() {
      builtin_interfaces::msg::Time out;
      out.sec = static_cast<std::int32_t>(state_time_sec);
      out.nanosec = static_cast<std::uint32_t>(
          (state_time_sec - static_cast<double>(out.sec)) * 1000000000.0);
      return out;
    }();

    wheel_leg_msgs::msg::ControlLoopDebug velocity_msg;
    velocity_msg.header.stamp = stamp;
    velocity_msg.loop_name = "velocity";
    velocity_msg.ref_primary = targets.target_distance;
    velocity_msg.now_primary = control_state.body.distance;
    velocity_msg.ref_secondary = targets.target_velocity;
    velocity_msg.now_secondary = control_state.body.velocity;
    velocity_debug_pub_->publish(velocity_msg);

    wheel_leg_msgs::msg::ControlLoopDebug yaw_msg;
    yaw_msg.header.stamp = stamp;
    yaw_msg.loop_name = "yaw";
    yaw_msg.ref_primary = targets.target_yaw_rate;
    yaw_msg.now_primary = control_state.body.yaw_rate;
    yaw_debug_pub_->publish(yaw_msg);

    wheel_leg_msgs::msg::ControlLoopDebug leg_msg;
    leg_msg.header.stamp = stamp;
    leg_msg.loop_name = "leg_length";
    leg_msg.ref_primary = targets.target_leg_length;
    leg_msg.now_primary = control_state.left_leg.leg_length;
    leg_msg.ref_secondary = targets.target_leg_length;
    leg_msg.now_secondary = control_state.right_leg.leg_length;
    leg_length_debug_pub_->publish(leg_msg);

    wheel_leg_msgs::msg::ControlLoopDebug anti_crash_msg;
    anti_crash_msg.header.stamp = stamp;
    anti_crash_msg.loop_name = "anti_crash";
    anti_crash_msg.ref_primary = 0.0;
    anti_crash_msg.now_primary =
        control_state.left_leg.phi - control_state.right_leg.phi;
    anti_crash_debug_pub_->publish(anti_crash_msg);

    wheel_leg_msgs::msg::ControlLoopDebug balance_msg;
    balance_msg.header.stamp = stamp;
    balance_msg.loop_name = "balance_deg";
    balance_msg.ref_primary = RadiansToDegrees(targets.target_phi);
    balance_msg.now_primary = RadiansToDegrees(
        0.5 * (control_state.left_leg.phi + control_state.right_leg.phi));
    balance_msg.ref_secondary = RadiansToDegrees(targets.target_pitch);
    balance_msg.now_secondary = RadiansToDegrees(control_state.body.pitch);
    balance_debug_pub_->publish(balance_msg);

    if (!ShouldPublishPlotTopics(state_time_sec)) {
      return;
    }

    velocity_plot_pub_->publish(velocity_msg);
    yaw_plot_pub_->publish(yaw_msg);
    leg_length_plot_pub_->publish(leg_msg);
    anti_crash_plot_pub_->publish(anti_crash_msg);
    balance_plot_pub_->publish(balance_msg);

    wheel_leg_msgs::msg::ControlLoopDebug velocity_filter_msg;
    velocity_filter_msg.header.stamp = stamp;
    velocity_filter_msg.loop_name = "velocity_ref_filter";
    velocity_filter_msg.ref_primary = raw_targets.target_velocity;
    velocity_filter_msg.now_primary = targets.target_velocity;
    velocity_filter_msg.ref_secondary = velocity_ref_lpf_rc_;
    velocity_filter_msg.now_secondary = velocity_ref_slew_rate_;
    velocity_ref_filter_plot_pub_->publish(velocity_filter_msg);

    wheel_leg_msgs::msg::ControlLoopDebug yaw_filter_msg;
    yaw_filter_msg.header.stamp = stamp;
    yaw_filter_msg.loop_name = "yaw_rate_ref_filter";
    yaw_filter_msg.ref_primary = raw_targets.target_yaw_rate;
    yaw_filter_msg.now_primary = targets.target_yaw_rate;
    yaw_filter_msg.ref_secondary = yaw_rate_ref_lpf_rc_;
    yaw_filter_msg.now_secondary = yaw_rate_ref_slew_rate_;
    yaw_rate_ref_filter_plot_pub_->publish(yaw_filter_msg);

    wheel_leg_msgs::msg::ControlLoopDebug leg_filter_msg;
    leg_filter_msg.header.stamp = stamp;
    leg_filter_msg.loop_name = "leg_length_ref_filter";
    leg_filter_msg.ref_primary = raw_targets.target_leg_length;
    leg_filter_msg.now_primary = targets.target_leg_length;
    leg_filter_msg.ref_secondary = leg_length_ref_lpf_rc_;
    leg_filter_msg.now_secondary = 0.0;
    leg_length_ref_filter_plot_pub_->publish(leg_filter_msg);
  }

  bool ShouldPublishPlotTopics(double state_time_sec) {
    if (debug_plot_publish_hz_ <= 0.0) {
      return false;
    }

    if (last_plot_publish_time_sec_ < 0.0) {
      last_plot_publish_time_sec_ = state_time_sec;
      return true;
    }

    const double plot_period_sec = 1.0 / debug_plot_publish_hz_;
    if (state_time_sec - last_plot_publish_time_sec_ + 1e-12 <
        plot_period_sec) {
      return false;
    }

    last_plot_publish_time_sec_ = state_time_sec;
    return true;
  }

  void PublishWheelEffortDebug(
      const wheel_leg_common::ControlCommand& command,
      double state_time_sec) {
    const auto stamp = [state_time_sec]() {
      builtin_interfaces::msg::Time out;
      out.sec = static_cast<std::int32_t>(state_time_sec);
      out.nanosec = static_cast<std::uint32_t>(
          (state_time_sec - static_cast<double>(out.sec)) * 1000000000.0);
      return out;
    }();

    wheel_leg_msgs::msg::ControlLoopDebug wheel_msg;
    wheel_msg.header.stamp = stamp;
    wheel_msg.loop_name = zero_hold_active_ ? "wheel_effort_zero_hold"
                                            : "wheel_effort";
    wheel_msg.ref_primary = zero_hold_active_ ? 1.0 : 0.0;
    wheel_msg.now_primary = FindEffortForJoint(command, "right_wheel");
    wheel_msg.ref_secondary = zero_hold_wheel_effort_limit_;
    wheel_msg.now_secondary = FindEffortForJoint(command, "left_wheel");
    wheel_effort_debug_pub_->publish(wheel_msg);

    if (ShouldPublishWheelPlotTopic(state_time_sec)) {
      wheel_effort_plot_pub_->publish(wheel_msg);
    }
  }

  void PublishTurnInternalDebug(
      const ControlStepOutputs& outputs,
      double state_time_sec) {
    const auto stamp = [state_time_sec]() {
      builtin_interfaces::msg::Time out;
      out.sec = static_cast<std::int32_t>(state_time_sec);
      out.nanosec = static_cast<std::uint32_t>(
          (state_time_sec - static_cast<double>(out.sec)) * 1000000000.0);
      return out;
    }();

    wheel_leg_msgs::msg::ControlLoopDebug turn_msg;
    turn_msg.header.stamp = stamp;
    turn_msg.loop_name = "turn_internal";
    turn_msg.ref_primary = outputs.steer_output;
    turn_msg.now_primary = outputs.swerving_speed_ff;
    turn_msg.ref_secondary = outputs.anti_crash_output;
    turn_msg.now_secondary =
        outputs.left_lqr_hip_torque - outputs.right_lqr_hip_torque;
    turn_internal_debug_pub_->publish(turn_msg);

    if (ShouldPublishTurnInternalPlotTopic(state_time_sec)) {
      turn_internal_plot_pub_->publish(turn_msg);
    }
  }

  void PublishLegLengthOutputDebug(
      const ControlStepOutputs& outputs,
      double state_time_sec) {
    const auto stamp = [state_time_sec]() {
      builtin_interfaces::msg::Time out;
      out.sec = static_cast<std::int32_t>(state_time_sec);
      out.nanosec = static_cast<std::uint32_t>(
          (state_time_sec - static_cast<double>(out.sec)) * 1000000000.0);
      return out;
    }();

    wheel_leg_msgs::msg::ControlLoopDebug leg_output_msg;
    leg_output_msg.header.stamp = stamp;
    leg_output_msg.loop_name = "leg_length_output";
    leg_output_msg.ref_primary = outputs.left_leg_length_force;
    leg_output_msg.now_primary = outputs.right_leg_length_force;
    leg_output_msg.ref_secondary = outputs.left_leg_length_force -
                                   outputs.right_leg_length_force;
    leg_output_msg.now_secondary = 0.5 * (outputs.left_leg_length_force +
                                          outputs.right_leg_length_force);
    leg_length_output_debug_pub_->publish(leg_output_msg);

    if (ShouldPublishLegLengthOutputPlotTopic(state_time_sec)) {
      leg_length_output_plot_pub_->publish(leg_output_msg);
    }
  }

  bool ShouldPublishLegLengthOutputPlotTopic(double state_time_sec) {
    if (debug_plot_publish_hz_ <= 0.0) {
      return false;
    }

    if (last_leg_length_output_plot_publish_time_sec_ < 0.0) {
      last_leg_length_output_plot_publish_time_sec_ = state_time_sec;
      return true;
    }

    const double plot_period_sec = 1.0 / debug_plot_publish_hz_;
    if (state_time_sec - last_leg_length_output_plot_publish_time_sec_ + 1e-12 <
        plot_period_sec) {
      return false;
    }

    last_leg_length_output_plot_publish_time_sec_ = state_time_sec;
    return true;
  }

  void PublishRollBalanceDebug(
      const ControlStepOutputs& outputs,
      const StandControlState& control_state,
      double state_time_sec) {
    const auto stamp = [state_time_sec]() {
      builtin_interfaces::msg::Time out;
      out.sec = static_cast<std::int32_t>(state_time_sec);
      out.nanosec = static_cast<std::uint32_t>(
          (state_time_sec - static_cast<double>(out.sec)) * 1000000000.0);
      return out;
    }();

    wheel_leg_msgs::msg::ControlLoopDebug roll_msg;
    roll_msg.header.stamp = stamp;
    roll_msg.loop_name = "roll_balance";
    roll_msg.ref_primary = 0.0;
    roll_msg.now_primary = control_state.body.roll;
    roll_msg.ref_secondary = control_state.body.roll_rate;
    roll_msg.now_secondary = outputs.roll_balance_output;
    roll_balance_debug_pub_->publish(roll_msg);

    if (ShouldPublishRollBalancePlotTopic(state_time_sec)) {
      roll_balance_plot_pub_->publish(roll_msg);
    }
  }

  bool ShouldPublishRollBalancePlotTopic(double state_time_sec) {
    if (debug_plot_publish_hz_ <= 0.0) {
      return false;
    }

    if (last_roll_balance_plot_publish_time_sec_ < 0.0) {
      last_roll_balance_plot_publish_time_sec_ = state_time_sec;
      return true;
    }

    const double plot_period_sec = 1.0 / debug_plot_publish_hz_;
    if (state_time_sec - last_roll_balance_plot_publish_time_sec_ + 1e-12 <
        plot_period_sec) {
      return false;
    }

    last_roll_balance_plot_publish_time_sec_ = state_time_sec;
    return true;
  }

  bool ShouldPublishTurnInternalPlotTopic(double state_time_sec) {
    if (debug_plot_publish_hz_ <= 0.0) {
      return false;
    }

    if (last_turn_internal_plot_publish_time_sec_ < 0.0) {
      last_turn_internal_plot_publish_time_sec_ = state_time_sec;
      return true;
    }

    const double plot_period_sec = 1.0 / debug_plot_publish_hz_;
    if (state_time_sec - last_turn_internal_plot_publish_time_sec_ + 1e-12 <
        plot_period_sec) {
      return false;
    }

    last_turn_internal_plot_publish_time_sec_ = state_time_sec;
    return true;
  }

  bool ShouldPublishWheelPlotTopic(double state_time_sec) {
    if (debug_plot_publish_hz_ <= 0.0) {
      return false;
    }

    if (last_wheel_plot_publish_time_sec_ < 0.0) {
      last_wheel_plot_publish_time_sec_ = state_time_sec;
      return true;
    }

    const double plot_period_sec = 1.0 / debug_plot_publish_hz_;
    if (state_time_sec - last_wheel_plot_publish_time_sec_ + 1e-12 <
        plot_period_sec) {
      return false;
    }

    last_wheel_plot_publish_time_sec_ = state_time_sec;
    return true;
  }

  void ClampCommand(wheel_leg_common::ControlCommand* command) {
    if (!command) {
      return;
    }

    for (auto& joint_effort : command->joint_efforts) {
      const double limit = EffortLimitForJoint(joint_effort.joint_name);
      const double clamped_effort =
          std::clamp(joint_effort.effort, -limit, limit);
      if (clamped_effort != joint_effort.effort) {
        if (!clamp_warning_logged_) {
          RCLCPP_WARN(
              get_logger(),
              "Clamped controller output for %s from %.3f to %.3f before publishing /joint_command",
              joint_effort.joint_name.c_str(),
              joint_effort.effort,
              clamped_effort);
          DumpTraceToFile("controller_output_clamped");
          clamp_warning_logged_ = true;
        }
        joint_effort.effort = clamped_effort;
      }
    }
  }

  void RecordTraceSample(
      double state_time_sec,
      double dt,
      const StandControlState& control_state,
      const wheel_leg_common::ControlCommand& raw_command,
      const wheel_leg_common::ControlCommand& clamped_command) {
    if (!trace_capture_enabled_) {
      return;
    }
    if (trace_capacity_ <= 0) {
      return;
    }
    if (trace_samples_.size() >= static_cast<std::size_t>(trace_capacity_)) {
      trace_samples_.pop_front();
    }
    trace_samples_.push_back(TraceSample{
        .state_time_sec = state_time_sec,
        .dt = dt,
        .state = control_state,
        .raw_command = raw_command,
        .clamped_command = clamped_command,
    });
  }

  void DumpTraceToFile(const std::string& reason) {
    if (!trace_capture_enabled_ || trace_dump_written_) {
      return;
    }
    std::ofstream output(trace_output_path_);
    if (!output.is_open()) {
      RCLCPP_WARN(
          get_logger(),
          "Failed to open trace_output_path=%s for controller trace dump",
          trace_output_path_.c_str());
      return;
    }

    output << std::fixed << std::setprecision(9);
    output << "reason,state_time_sec,dt,"
           << "body_distance,body_velocity,body_roll,body_roll_rate,body_pitch,body_pitch_rate,body_yaw_rate,"
           << "left_leg_length,left_phi,right_leg_length,right_phi,"
           << "raw_right_hip,raw_right_knee,raw_left_hip,raw_left_knee,raw_right_wheel,raw_left_wheel,"
           << "clamped_right_hip,clamped_right_knee,clamped_left_hip,clamped_left_knee,clamped_right_wheel,clamped_left_wheel\n";

    for (const auto& sample : trace_samples_) {
      output << reason << ','
             << sample.state_time_sec << ','
             << sample.dt << ','
             << sample.state.body.distance << ','
             << sample.state.body.velocity << ','
             << sample.state.body.roll << ','
             << sample.state.body.roll_rate << ','
             << sample.state.body.pitch << ','
             << sample.state.body.pitch_rate << ','
             << sample.state.body.yaw_rate << ','
             << sample.state.left_leg.leg_length << ','
             << sample.state.left_leg.phi << ','
             << sample.state.right_leg.leg_length << ','
             << sample.state.right_leg.phi << ','
             << FindEffortForJoint(sample.raw_command, "right_hip") << ','
             << FindEffortForJoint(sample.raw_command, "right_knee") << ','
             << FindEffortForJoint(sample.raw_command, "left_hip") << ','
             << FindEffortForJoint(sample.raw_command, "left_knee") << ','
             << FindEffortForJoint(sample.raw_command, "right_wheel") << ','
             << FindEffortForJoint(sample.raw_command, "left_wheel") << ','
             << FindEffortForJoint(sample.clamped_command, "right_hip") << ','
             << FindEffortForJoint(sample.clamped_command, "right_knee") << ','
             << FindEffortForJoint(sample.clamped_command, "left_hip") << ','
             << FindEffortForJoint(sample.clamped_command, "left_knee") << ','
             << FindEffortForJoint(sample.clamped_command, "right_wheel") << ','
             << FindEffortForJoint(sample.clamped_command, "left_wheel") << '\n';
    }

    trace_dump_written_ = true;
    RCLCPP_INFO(
        get_logger(),
        "Wrote controller trace dump with %zu samples to %s",
        trace_samples_.size(),
        trace_output_path_.c_str());
  }

  double EffortLimitForJoint(const std::string& joint_name) const {
    if (joint_name.find("wheel") != std::string::npos) {
      return wheel_effort_limit_;
    }
    if (joint_name.find("knee") != std::string::npos) {
      return knee_effort_limit_;
    }
    return hip_effort_limit_;
  }

  bool publish_control_command_ = false;
  StandControlStageConfig stage_config_;
  bool clamp_warning_logged_ = false;
  bool has_processed_state_ = false;
  bool invalid_dt_warning_logged_ = false;
  bool out_of_range_dt_warning_logged_ = false;
  bool non_increasing_stamp_warning_logged_ = false;
  bool trace_capture_enabled_ = true;
  bool trace_dump_written_ = false;
  bool disabled_stops_publishing_ = true;
  bool fallback_active_ = false;
  bool zero_hold_active_ = false;
  bool zero_hold_latched_ = false;
  bool zero_hold_distance_ref_initialized_ = false;
  std::size_t warmup_sample_count_ = 0;
  std::size_t dt_debug_log_count_ = 0;
  double last_processed_state_time_sec_ =
      -std::numeric_limits<double>::infinity();
  double hip_effort_limit_ = 50.0;
  double knee_effort_limit_ = 50.0;
  double wheel_effort_limit_ = 20.0;
  double command_timeout_sec_ = 0.0;
  double target_velocity_scale_ = 0.0;
  double target_yaw_rate_scale_ = 0.0;
  double yaw_rate_assist_scale_ = 0.0;
  double turn_hip_feedforward_scale_ = DefaultTurnHipFeedforwardScale();
  double body_height_offset_scale_ = 0.0;
  double target_leg_length_min_ = 0.0;
  double target_leg_length_max_ = 0.0;
  double target_phi_deg_ = 97.1;
  double target_phi_min_deg_ = 30.0;
  double target_phi_max_deg_ = 150.0;
  double target_pitch_deg_ = 0.0;
  double velocity_ref_lpf_rc_ = 0.0;
  double velocity_ref_slew_rate_ = 0.0;
  double velocity_snap_threshold_ = 0.0;
  double velocity_zero_hold_threshold_ = 0.0;
  double velocity_zero_hold_exit_threshold_ = 0.0;
  double velocity_zero_hold_entry_delay_sec_ = 0.0;
  double zero_hold_measurement_clamp_threshold_ = 0.0;
  double zero_hold_wheel_effort_limit_ = 0.0;
  double yaw_rate_ref_lpf_rc_ = 0.0;
  double yaw_rate_ref_slew_rate_ = 0.0;
  double leg_length_ref_lpf_rc_ = 0.0;
  double debug_plot_publish_hz_ = 0.0;
  double expected_dt_sec_ = kDefaultExpectedDtSec;
  double accepted_dt_tolerance_sec_ = kDefaultAcceptedDtToleranceSec;
  double velocity_mode_distance_ref_ = 0.0;
  double zero_hold_distance_ref_ = 0.0;
  double last_plot_publish_time_sec_ = -1.0;
  double last_wheel_plot_publish_time_sec_ = -1.0;
  double last_turn_internal_plot_publish_time_sec_ = -1.0;
  double last_leg_length_output_plot_publish_time_sec_ = -1.0;
  double last_roll_balance_plot_publish_time_sec_ = -1.0;
  double zero_hold_candidate_start_time_sec_ = -1.0;
  double last_cmd_vel_time_sec_ = -1.0;
  double last_control_mode_time_sec_ = -1.0;
  double last_body_cmd_time_sec_ = -1.0;
  double last_rc_status_time_sec_ = -1.0;
  int trace_capacity_ = static_cast<int>(kDefaultTraceCapacity);
  std::string trace_output_path_ = kDefaultTracePath;
  std::string latest_control_mode_;
  std::optional<Mode> last_effective_mode_;
  std::optional<std::string> last_arbitration_reason_;
  std::optional<wheel_leg_msgs::msg::RcStatus> latest_rc_status_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  ControlTargets default_targets_;
  StandLegacyPidDefaults pid_defaults_;
  TargetFilterState filter_state_;
  geometry_msgs::msg::Twist latest_cmd_vel_;
  wheel_leg_msgs::msg::BodyCommand latest_body_cmd_;
  std::deque<TraceSample> trace_samples_;
  ControllerOrchestrator orchestrator_;
  rclcpp::Subscription<wheel_leg_msgs::msg::StandControlState>::SharedPtr
      control_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_sub_;
  rclcpp::Subscription<wheel_leg_msgs::msg::BodyCommand>::SharedPtr
      body_cmd_sub_;
  rclcpp::Subscription<wheel_leg_msgs::msg::RcStatus>::SharedPtr
      rc_status_sub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::JointCommand>::SharedPtr
      joint_command_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      velocity_debug_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      yaw_debug_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      leg_length_debug_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      leg_length_output_debug_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      anti_crash_debug_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      roll_balance_debug_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      velocity_plot_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      yaw_plot_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      leg_length_plot_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      leg_length_output_plot_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      anti_crash_plot_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      roll_balance_plot_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      wheel_effort_debug_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      wheel_effort_plot_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      balance_debug_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      balance_plot_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      turn_internal_debug_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      turn_internal_plot_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      velocity_ref_filter_plot_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      yaw_rate_ref_filter_plot_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      leg_length_ref_filter_plot_pub_;
};

}  // namespace wheel_leg_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheel_leg_control::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
