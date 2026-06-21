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

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "wheel_leg_bridge/message_conversions.hpp"
#include "wheel_leg_control/controller_orchestrator.hpp"
#include "wheel_leg_control/stand_runtime_defaults.hpp"
#include "wheel_leg_control/state_message_conversions.hpp"
#include "wheel_leg_msgs/msg/body_command.hpp"
#include "wheel_leg_msgs/msg/rc_status.hpp"

namespace wheel_leg_control {
namespace {

constexpr double kMinAcceptedDtSec = 0.0015;
constexpr double kMaxAcceptedDtSec = 0.0035;
constexpr std::size_t kWarmupSamplesRequired = 3;
constexpr std::size_t kDtDebugSamplesToLog = 5;
constexpr std::size_t kDefaultTraceCapacity = 300;
constexpr const char* kDefaultTracePath = "/tmp/wheel_leg_takeover_trace.csv";

constexpr const char* kModeStand = "stand";
constexpr const char* kModeVelocity = "velocity";
constexpr const char* kModeDisabled = "disabled";

double ToSec(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
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
    publish_control_command_ =
        declare_parameter<bool>("publish_control_command", true);
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
        declare_parameter<double>("target_yaw_rate_scale", 1.2);
    yaw_rate_assist_scale_ =
        declare_parameter<double>("yaw_rate_assist_scale", 1.0);
    body_height_offset_scale_ =
        declare_parameter<double>("body_height_offset_scale", 0.05);
    target_leg_length_min_ =
        declare_parameter<double>("target_leg_length_min", 0.22);
    target_leg_length_max_ =
        declare_parameter<double>("target_leg_length_max", 0.30);
    disabled_stops_publishing_ =
        declare_parameter<bool>("disabled_stops_publishing", true);
    declare_parameter<double>("publish_rate_hz", 500.0);
    RCLCPP_WARN(
        get_logger(),
        "Parameter publish_rate_hz is ignored; /wheel_leg_controller now runs one control step per new /robot_state sample.");

    latest_control_mode_ = kModeStand;

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
  }

 private:
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

    if (raw_dt < kMinAcceptedDtSec || raw_dt > kMaxAcceptedDtSec) {
      if (!out_of_range_dt_warning_logged_) {
        RCLCPP_WARN(
            get_logger(),
            "Ignoring /robot_state sample with out-of-range dt=%.9f; expected about 0.002 s",
            raw_dt);
        out_of_range_dt_warning_logged_ = true;
      }
      return;
    }

    const double dt = std::clamp(raw_dt, kMinAcceptedDtSec, kMaxAcceptedDtSec);
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
    ApplyArbitration(arbitration, control_state, state_time_sec);

    if (!publish_control_command_) {
      return;
    }

    if (last_effective_mode_.has_value() &&
        *last_effective_mode_ == Mode::kDisabled &&
        !disabled_stops_publishing_) {
      const auto zero_command = BuildZeroCommand(state_time_sec);
      joint_command_pub_->publish(
          wheel_leg_bridge::ToRosJointCommand(zero_command));
      return;
    }

    if (arbitration.hold_output) {
      return;
    }

    const auto command =
        orchestrator_.Step(state_time_sec, dt, control_state);
    if (!command.has_value()) {
      return;
    }

    auto limited_command = *command;
    ClampCommand(&limited_command);
    RecordTraceSample(
        state_time_sec, dt, control_state, *command, limited_command);
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

  void ApplyArbitration(const ArbitrationResult& arbitration,
                        const StandControlState& control_state,
                        double state_time_sec) {
    orchestrator_.SetTargets(arbitration.targets);

    const bool mode_changed = !last_effective_mode_.has_value() ||
                              arbitration.mode != *last_effective_mode_;
    const bool reason_changed =
        !last_arbitration_reason_.has_value() ||
        arbitration.reason != *last_arbitration_reason_;
    const bool fallback_active = !IsOperationalModeReason(arbitration.reason);
    const bool fallback_changed = fallback_active_ != fallback_active;

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
           << "body_distance,body_velocity,body_pitch,body_pitch_rate,body_yaw_rate,"
           << "left_leg_length,left_phi,right_leg_length,right_phi,"
           << "raw_right_hip,raw_right_knee,raw_left_hip,raw_left_knee,raw_right_wheel,raw_left_wheel,"
           << "clamped_right_hip,clamped_right_knee,clamped_left_hip,clamped_left_knee,clamped_right_wheel,clamped_left_wheel\n";

    for (const auto& sample : trace_samples_) {
      output << reason << ','
             << sample.state_time_sec << ','
             << sample.dt << ','
             << sample.state.body.distance << ','
             << sample.state.body.velocity << ','
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
  bool clamp_warning_logged_ = false;
  bool has_processed_state_ = false;
  bool invalid_dt_warning_logged_ = false;
  bool out_of_range_dt_warning_logged_ = false;
  bool non_increasing_stamp_warning_logged_ = false;
  bool trace_capture_enabled_ = true;
  bool trace_dump_written_ = false;
  bool disabled_stops_publishing_ = true;
  bool fallback_active_ = false;
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
  double body_height_offset_scale_ = 0.0;
  double target_leg_length_min_ = 0.0;
  double target_leg_length_max_ = 0.0;
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
  ControlTargets default_targets_;
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
};

}  // namespace wheel_leg_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheel_leg_control::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
