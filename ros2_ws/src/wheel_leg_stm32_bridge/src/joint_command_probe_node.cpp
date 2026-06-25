#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "wheel_leg_hw/interface_contract.hpp"
#include "wheel_leg_msgs/msg/joint_command.hpp"

namespace wheel_leg_stm32_bridge {
namespace {

constexpr std::size_t kJointCount = 6;
constexpr double kDefaultPublishRateHz = 20.0;
constexpr double kDefaultEffortNm = 0.2;
constexpr double kDefaultSquarePeriodSec = 2.0;

class JointCommandProbeNode : public rclcpp::Node {
 public:
  JointCommandProbeNode() : rclcpp::Node("joint_command_probe_node") {
    joint_name_ = declare_parameter<std::string>("joint_name", "left_hip");
    effort_nm_ = declare_parameter<double>("effort_nm", kDefaultEffortNm);
    publish_rate_hz_ =
        declare_parameter<double>("publish_rate_hz", kDefaultPublishRateHz);
    mode_ = declare_parameter<std::string>("mode", "constant");
    square_period_sec_ =
        declare_parameter<double>("square_period_sec", kDefaultSquarePeriodSec);
    start_with_zero_ = declare_parameter<bool>("start_with_zero", true);

    publisher_ = create_publisher<wheel_leg_msgs::msg::JointCommand>(
        "/joint_command", rclcpp::SystemDefaultsQoS());

    if (!ResolveJointIndex()) {
      throw std::runtime_error("invalid joint_name for joint_command_probe_node");
    }
    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be > 0");
    }
    if (square_period_sec_ <= 0.0) {
      throw std::runtime_error("square_period_sec must be > 0");
    }

    start_time_ = now();
    publish_zero_once_ = start_with_zero_;
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz_)),
        [this]() { PublishCommand(); });

    RCLCPP_INFO(
        get_logger(),
        "Publishing /joint_command probe: joint=%s mode=%s effort=%.3fNm rate=%.1fHz",
        joint_name_.c_str(),
        mode_.c_str(),
        effort_nm_,
        publish_rate_hz_);
  }

  ~JointCommandProbeNode() override { PublishZeroCommand(); }

 private:
  bool ResolveJointIndex() {
    const auto it = std::find(
        wheel_leg_hw::kCanonicalJointNames.begin(),
        wheel_leg_hw::kCanonicalJointNames.end(),
        joint_name_);
    if (it == wheel_leg_hw::kCanonicalJointNames.end()) {
      std::string valid_names;
      for (std::size_t i = 0; i < wheel_leg_hw::kCanonicalJointNames.size(); ++i) {
        if (!valid_names.empty()) {
          valid_names += ", ";
        }
        valid_names += std::string(wheel_leg_hw::kCanonicalJointNames[i]);
      }
      RCLCPP_ERROR(
          get_logger(),
          "Unknown joint_name '%s'. Valid values: %s",
          joint_name_.c_str(),
          valid_names.c_str());
      return false;
    }

    joint_index_ = static_cast<std::size_t>(
        std::distance(wheel_leg_hw::kCanonicalJointNames.begin(), it));
    return true;
  }

  double ComputeCommandEffort() {
    if (publish_zero_once_) {
      return 0.0;
    }
    if (mode_ == "constant") {
      return effort_nm_;
    }
    if (mode_ == "square") {
      const double elapsed_sec = (now() - start_time_).seconds();
      const double phase =
          std::fmod(elapsed_sec, square_period_sec_) / square_period_sec_;
      return phase < 0.5 ? effort_nm_ : -effort_nm_;
    }

    if (!warned_unknown_mode_) {
      RCLCPP_WARN(
          get_logger(),
          "Unknown mode '%s', falling back to constant",
          mode_.c_str());
      warned_unknown_mode_ = true;
    }
    return effort_nm_;
  }

  wheel_leg_msgs::msg::JointCommand BuildCommand(double selected_effort) const {
    wheel_leg_msgs::msg::JointCommand msg;
    msg.header.stamp = now();
    msg.joint_names.reserve(kJointCount);
    msg.efforts.assign(kJointCount, 0.0);

    for (const auto& joint : wheel_leg_hw::kCanonicalJointNames) {
      msg.joint_names.push_back(std::string(joint));
    }
    msg.efforts[joint_index_] = selected_effort;
    return msg;
  }

  void PublishCommand() {
    const double selected_effort = ComputeCommandEffort();
    publisher_->publish(BuildCommand(selected_effort));
    publish_zero_once_ = false;
  }

  void PublishZeroCommand() {
    if (!publisher_) {
      return;
    }
    publisher_->publish(BuildCommand(0.0));
  }

  std::string joint_name_;
  double effort_nm_ = 0.0;
  double publish_rate_hz_ = 0.0;
  std::string mode_;
  double square_period_sec_ = 0.0;
  bool start_with_zero_ = true;
  bool publish_zero_once_ = false;
  bool warned_unknown_mode_ = false;
  std::size_t joint_index_ = 0;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<wheel_leg_msgs::msg::JointCommand>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace wheel_leg_stm32_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<wheel_leg_stm32_bridge::JointCommandProbeNode>());
  rclcpp::shutdown();
  return 0;
}
