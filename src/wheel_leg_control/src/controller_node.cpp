#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "wheel_leg_bridge/message_conversions.hpp"
#include "wheel_leg_control/controller_orchestrator.hpp"
#include "wheel_leg_control/state_assembler.hpp"

namespace wheel_leg_control {

class ControllerNode : public rclcpp::Node {
 public:
  ControllerNode()
      : rclcpp::Node("wheel_leg_controller") {
    publish_zero_effort_command_ =
        declare_parameter<bool>("publish_zero_effort_command", false);
    double publish_rate_hz =
        declare_parameter<double>("publish_rate_hz", 100.0);
    if (publish_rate_hz <= 0.0) {
      publish_rate_hz = 100.0;
    }
    const auto period =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          assembler_.UpdateJointState(
              wheel_leg_bridge::FromRosJointState(*msg));
        });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          assembler_.UpdateImu(wheel_leg_bridge::FromRosImu(*msg));
        });
    joint_command_pub_ =
        create_publisher<wheel_leg_msgs::msg::JointCommand>(
            "/joint_command", rclcpp::SystemDefaultsQoS());
    timer_ = create_wall_timer(period, [this]() { OnTimer(); });
  }

 private:
  void OnTimer() {
    if (!publish_zero_effort_command_ || !assembler_.HasCompleteState()) {
      return;
    }

    const auto command = orchestrator_.Step(assembler_.BuildSnapshot());
    if (!command.has_value()) {
      return;
    }

    joint_command_pub_->publish(
        wheel_leg_bridge::ToRosJointCommand(*command));
  }

  bool publish_zero_effort_command_ = false;
  StateAssembler assembler_;
  ControllerOrchestrator orchestrator_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::JointCommand>::SharedPtr
      joint_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace wheel_leg_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheel_leg_control::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
