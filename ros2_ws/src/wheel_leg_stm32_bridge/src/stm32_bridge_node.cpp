#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "wheel_leg_msgs/msg/joint_command.hpp"
#include "wheel_leg_msgs/msg/stand_control_state.hpp"

namespace wheel_leg_stm32_bridge {

class Stm32BridgeNode : public rclcpp::Node {
 public:
  Stm32BridgeNode() : rclcpp::Node("wheel_leg_stm32_bridge") {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 500.0);
    nominal_leg_length_ = declare_parameter<double>("nominal_leg_length", 0.28);

    robot_state_pub_ = create_publisher<wheel_leg_msgs::msg::StandControlState>(
        "/robot_state", rclcpp::SystemDefaultsQoS());
    joint_command_sub_ = create_subscription<wheel_leg_msgs::msg::JointCommand>(
        "/joint_command", rclcpp::SystemDefaultsQoS(),
        [this](const wheel_leg_msgs::msg::JointCommand::SharedPtr msg) {
          last_joint_command_stamp_ = now();
          RCLCPP_DEBUG(
              get_logger(),
              "Received /joint_command with %zu joint effort entries",
              msg->joint_names.size());
        });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { PublishPlaceholderRobotState(); });

    RCLCPP_WARN(
        get_logger(),
        "wheel_leg_stm32_bridge is running in placeholder mode and publishes a synthetic /robot_state at %.1f Hz",
        publish_rate_hz_);
  }

 private:
  void PublishPlaceholderRobotState() {
    wheel_leg_msgs::msg::StandControlState state;
    state.header.stamp = now();
    state.header.frame_id = "base_link";
    state.left_leg_length = nominal_leg_length_;
    state.right_leg_length = nominal_leg_length_;
    robot_state_pub_->publish(state);
  }

  double publish_rate_hz_ = 500.0;
  double nominal_leg_length_ = 0.28;
  rclcpp::Time last_joint_command_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<wheel_leg_msgs::msg::StandControlState>::SharedPtr
      robot_state_pub_;
  rclcpp::Subscription<wheel_leg_msgs::msg::JointCommand>::SharedPtr
      joint_command_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace wheel_leg_stm32_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<wheel_leg_stm32_bridge::Stm32BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
