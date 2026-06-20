#include "ros2_bridge.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "sim_adapter.h"

#ifdef WHEEL_LEG_ENABLE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wheel_leg_bridge/message_conversions.hpp>
#include <wheel_leg_msgs/msg/joint_command.hpp>
#endif

namespace wheel_leg {
namespace {

constexpr double kStatePublishPeriodSec = 0.01;
constexpr double kCommandTimeoutSec = 0.2;
constexpr const char* kNodeName = "mujoco_bridge";

#ifdef WHEEL_LEG_ENABLE_ROS2

wheel_leg_common::TimePoint ToCommonTime(double sim_time) {
  wheel_leg_common::TimePoint stamp;
  const double clamped_time = std::max(0.0, sim_time);
  stamp.sec = static_cast<int32_t>(std::floor(clamped_time));
  stamp.nanosec = static_cast<uint32_t>(
      (clamped_time - static_cast<double>(stamp.sec)) * 1000000000.0);
  return stamp;
}

class Ros2Bridge {
 public:
  Ros2Bridge() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
      owns_rclcpp_context_ = true;
    }

    node_ = std::make_shared<rclcpp::Node>(kNodeName);
    enable_ros_command_ =
        node_->declare_parameter<bool>("enable_ros_command", false);
    joint_state_pub_ =
        node_->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SystemDefaultsQoS());
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SystemDefaultsQoS());
    command_sub_ =
        node_->create_subscription<wheel_leg_msgs::msg::JointCommand>(
            "/joint_command", rclcpp::SystemDefaultsQoS(),
            [this](wheel_leg_msgs::msg::JointCommand::SharedPtr msg) {
              latest_command_ = std::move(msg);
              pending_command_update_ = true;
            });

    std::cout << "ROS2 MuJoCo bridge ready: node=" << kNodeName
              << ", topics=/joint_states,/imu,/joint_command"
              << ", enable_ros_command=false" << std::endl;
  }

  ~Ros2Bridge() {
    node_.reset();
    if (owns_rclcpp_context_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void Spin(double sim_time) {
    if (!node_ || !rclcpp::ok()) {
      return;
    }
    rclcpp::spin_some(node_);
    enable_ros_command_ = node_->get_parameter("enable_ros_command").as_bool();
    if (pending_command_update_) {
      pending_command_update_ = false;
      latest_command_sim_time_ = sim_time;
      has_command_ = true;
    }
  }

  void PublishState(const mjModel* m, const mjData* d) {
    if (!node_ || !rclcpp::ok() || !m || !d) {
      return;
    }
    if (next_publish_time_ < 0.0) {
      next_publish_time_ = d->time;
    }
    if (d->time + 1e-9 < next_publish_time_) {
      return;
    }
    PublishJointState(m, d);
    PublishImu(m, d);
    next_publish_time_ += kStatePublishPeriodSec;
    if (next_publish_time_ <= d->time) {
      next_publish_time_ = d->time + kStatePublishPeriodSec;
    }
  }

  void ApplyCommand(const mjModel* m, mjData* d) {
    if (!node_ || !rclcpp::ok() || !m || !d || !enable_ros_command_ ||
        !has_command_) {
      return;
    }
    if (d->time - latest_command_sim_time_ > kCommandTimeoutSec) {
      if (!command_timeout_logged_) {
        RCLCPP_WARN(node_->get_logger(),
                    "/joint_command timed out after %.3f s; actuator writes are suspended",
                    kCommandTimeoutSec);
        command_timeout_logged_ = true;
      }
      return;
    }
    if (!ValidateCommand()) {
      if (latest_command_ != last_invalid_command_logged_) {
        RCLCPP_WARN(node_->get_logger(),
                    "Rejected invalid /joint_command; no actuator was written");
        last_invalid_command_logged_ = latest_command_;
      }
      return;
    }

    wheel_leg_common::ControlCommand command;
    command.stamp = ToCommonTime(d->time);
    command.joint_efforts.reserve(latest_command_->joint_names.size());
    for (std::size_t i = 0; i < latest_command_->joint_names.size(); ++i) {
      wheel_leg_common::JointEffortCommand joint_effort;
      joint_effort.joint_name = latest_command_->joint_names[i];
      joint_effort.effort = latest_command_->efforts[i];
      command.joint_efforts.push_back(std::move(joint_effort));
    }

    const ApplyCommandResult result = ApplyControlCommand(m, d, command);
    if (!result.accepted) {
      if (latest_command_ != last_invalid_command_logged_) {
        RCLCPP_WARN(node_->get_logger(),
                    "Rejected /joint_command for unsupported joint '%s'; no actuator was written",
                    result.rejected_joint_name.c_str());
        last_invalid_command_logged_ = latest_command_;
      }
      return;
    }

    if (result.command_was_clamped &&
        latest_command_ != last_clamped_command_logged_) {
      RCLCPP_WARN(node_->get_logger(),
                  "Clamped /joint_command for %s from %.3f to %.3f",
                  result.first_clamped_joint_name.c_str(),
                  result.first_requested_effort,
                  result.first_applied_effort);
      last_clamped_command_logged_ = latest_command_;
    }
    if (latest_command_ != last_applied_command_logged_) {
      RCLCPP_INFO(node_->get_logger(),
                  "Applied /joint_command to %zu actuator(s)",
                  result.applied_effort_count);
      last_applied_command_logged_ = latest_command_;
    }
    command_timeout_logged_ = false;
  }

 private:
  void PublishJointState(const mjModel* m, const mjData* d) {
    const wheel_leg_common::JointStateSample sample = SampleJointState(m, d);
    joint_state_pub_->publish(wheel_leg_bridge::ToRosJointState(sample));
  }

  void PublishImu(const mjModel* m, const mjData* d) {
    const wheel_leg_common::ImuSample sample = SampleImu(m, d);
    imu_pub_->publish(wheel_leg_bridge::ToRosImu(sample));
  }

  bool ValidateCommand() {
    if (!latest_command_) {
      return false;
    }
    if (latest_command_->joint_names.size() != latest_command_->efforts.size()) {
      return false;
    }
    for (std::size_t i = 0; i < latest_command_->joint_names.size(); ++i) {
      if (!std::isfinite(latest_command_->efforts[i])) {
        return false;
      }
      if (!wheel_leg_common::IsKnownJointName(latest_command_->joint_names[i])) {
        return false;
      }
    }
    return true;
  }

  bool owns_rclcpp_context_ = false;
  bool enable_ros_command_ = false;
  bool pending_command_update_ = false;
  bool has_command_ = false;
  bool command_timeout_logged_ = false;
  double latest_command_sim_time_ = 0.0;
  double next_publish_time_ = -1.0;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<wheel_leg_msgs::msg::JointCommand>::SharedPtr
      command_sub_;
  wheel_leg_msgs::msg::JointCommand::SharedPtr latest_command_;
  wheel_leg_msgs::msg::JointCommand::SharedPtr last_applied_command_logged_;
  wheel_leg_msgs::msg::JointCommand::SharedPtr last_clamped_command_logged_;
  wheel_leg_msgs::msg::JointCommand::SharedPtr last_invalid_command_logged_;
};

std::unique_ptr<Ros2Bridge> g_bridge;

#endif  // WHEEL_LEG_ENABLE_ROS2

}  // namespace

void InitializeRos2Bridge(const mjModel* m, const mjData* d) {
#ifdef WHEEL_LEG_ENABLE_ROS2
  (void)m;
  (void)d;
  if (!g_bridge) {
    g_bridge = std::make_unique<Ros2Bridge>();
  }
#else
  (void)m;
  (void)d;
#endif
}

void SpinRos2Bridge(const mjModel* m, const mjData* d) {
#ifdef WHEEL_LEG_ENABLE_ROS2
  (void)m;
  if (g_bridge && d) {
    g_bridge->Spin(d->time);
  }
#else
  (void)m;
  (void)d;
#endif
}

void ApplyRos2Command(const mjModel* m, mjData* d) {
#ifdef WHEEL_LEG_ENABLE_ROS2
  if (g_bridge) {
    g_bridge->ApplyCommand(m, d);
  }
#else
  (void)m;
  (void)d;
#endif
}

void PublishRos2State(const mjModel* m, const mjData* d) {
#ifdef WHEEL_LEG_ENABLE_ROS2
  if (g_bridge) {
    g_bridge->PublishState(m, d);
  }
#else
  (void)m;
  (void)d;
#endif
}

}  // namespace wheel_leg
