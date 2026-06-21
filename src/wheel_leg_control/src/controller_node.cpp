#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "wheel_leg_bridge/message_conversions.hpp"
#include "wheel_leg_control/controller_orchestrator.hpp"
#include "wheel_leg_control/state_message_conversions.hpp"

namespace wheel_leg_control {
namespace {

constexpr double kMinAcceptedDtSec = 0.0015;
constexpr double kMaxAcceptedDtSec = 0.0035;
constexpr std::size_t kWarmupSamplesRequired = 3;
constexpr std::size_t kDtDebugSamplesToLog = 5;
constexpr std::size_t kDefaultTraceCapacity = 300;
constexpr const char* kDefaultTracePath = "/tmp/wheel_leg_takeover_trace.csv";

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

}  // namespace

class ControllerNode : public rclcpp::Node {
 public:
  ControllerNode()
      : rclcpp::Node("wheel_leg_controller") {
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
    declare_parameter<double>("publish_rate_hz", 500.0);
    RCLCPP_WARN(
        get_logger(),
        "Parameter publish_rate_hz is ignored; /wheel_leg_controller now runs one control step per new /robot_state sample.");

    control_state_sub_ =
        create_subscription<wheel_leg_msgs::msg::StandControlState>(
            "/robot_state", rclcpp::SystemDefaultsQoS(),
            [this](const wheel_leg_msgs::msg::StandControlState::SharedPtr msg) {
              OnControlState(*msg);
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

    const double dt =
        std::clamp(raw_dt, kMinAcceptedDtSec, kMaxAcceptedDtSec);
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
        orchestrator_.ResetForState(control_state);
      }
      return;
    }

    if (!publish_control_command_) {
      return;
    }

    const auto command =
        orchestrator_.Step(state_time_sec, dt, control_state);
    if (!command.has_value()) {
      return;
    }

    auto limited_command = *command;
    ClampCommand(&limited_command);
    RecordTraceSample(state_time_sec, dt, control_state, *command, limited_command);
    joint_command_pub_->publish(
        wheel_leg_bridge::ToRosJointCommand(limited_command));
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
  std::size_t warmup_sample_count_ = 0;
  std::size_t dt_debug_log_count_ = 0;
  double last_processed_state_time_sec_ =
      -std::numeric_limits<double>::infinity();
  double hip_effort_limit_ = 50.0;
  double knee_effort_limit_ = 50.0;
  double wheel_effort_limit_ = 20.0;
  bool trace_capture_enabled_ = true;
  bool trace_dump_written_ = false;
  int trace_capacity_ = static_cast<int>(kDefaultTraceCapacity);
  std::string trace_output_path_ = kDefaultTracePath;
  std::deque<TraceSample> trace_samples_;
  ControllerOrchestrator orchestrator_;
  rclcpp::Subscription<wheel_leg_msgs::msg::StandControlState>::SharedPtr
      control_state_sub_;
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
