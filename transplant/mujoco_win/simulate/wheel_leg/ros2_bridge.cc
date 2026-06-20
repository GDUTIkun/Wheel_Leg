#include "ros2_bridge.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "sensor.h"

#ifdef WHEEL_LEG_ENABLE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wheel_leg_msgs/msg/joint_command.hpp>
#endif

namespace wheel_leg {
namespace {

constexpr double kStatePublishPeriodSec = 0.01;
constexpr double kCommandTimeoutSec = 0.2;
constexpr const char* kNodeName = "mujoco_bridge";
constexpr const char* kBaseFrameId = "base_link";

struct JointMapping {
  const char* ros_name;
  const char* mujoco_joint;
  const char* mujoco_actuator;
};

constexpr std::array<JointMapping, 6> kJointMappings = {{
    {"left_hip", "left_hip_joint", "left_hip_motor"},
    {"left_knee", "left_knee_joint", "left_knee_motor"},
    {"left_wheel", "left_wheel_joint", "left_wheel_motor"},
    {"right_hip", "right_hip_joint", "right_hip_motor"},
    {"right_knee", "right_knee_joint", "right_knee_motor"},
    {"right_wheel", "right_wheel_joint", "right_wheel_motor"},
}};

#ifdef WHEEL_LEG_ENABLE_ROS2

builtin_interfaces::msg::Time ToRosTime(double sim_time) {
  builtin_interfaces::msg::Time stamp;
  const double clamped_time = std::max(0.0, sim_time);
  stamp.sec = static_cast<int32_t>(std::floor(clamped_time));
  stamp.nanosec =
      static_cast<uint32_t>((clamped_time - stamp.sec) * 1000000000.0);
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
    if (!ValidateCommand(m)) {
      if (latest_command_ != last_invalid_command_logged_) {
        RCLCPP_WARN(node_->get_logger(),
                    "Rejected invalid /joint_command; no actuator was written");
        last_invalid_command_logged_ = latest_command_;
      }
      return;
    }

    bool command_was_clamped = false;
    std::string clamped_joint_name;
    double first_requested_effort = 0.0;
    double first_clamped_effort = 0.0;
    for (std::size_t i = 0; i < latest_command_->joint_names.size(); ++i) {
      const std::string& joint_name = latest_command_->joint_names[i];
      const int actuator_id = actuator_ids_by_ros_name_[joint_name];
      double effort = latest_command_->efforts[i];
      if (m->actuator_ctrllimited[actuator_id]) {
        const double min_ctrl = m->actuator_ctrlrange[2 * actuator_id];
        const double max_ctrl = m->actuator_ctrlrange[2 * actuator_id + 1];
        const double requested_effort = effort;
        effort = std::clamp(effort, min_ctrl, max_ctrl);
        if (effort != requested_effort) {
          if (!command_was_clamped) {
            clamped_joint_name = joint_name;
            first_requested_effort = requested_effort;
            first_clamped_effort = effort;
          }
          command_was_clamped = true;
        }
      }
      d->ctrl[actuator_id] = effort;
    }
    if (command_was_clamped &&
        latest_command_ != last_clamped_command_logged_) {
      RCLCPP_WARN(node_->get_logger(),
                  "Clamped /joint_command for %s from %.3f to %.3f",
                  clamped_joint_name.c_str(), first_requested_effort,
                  first_clamped_effort);
      last_clamped_command_logged_ = latest_command_;
    }
    if (latest_command_ != last_applied_command_logged_) {
      RCLCPP_INFO(node_->get_logger(),
                  "Applied /joint_command to %zu actuator(s)",
                  latest_command_->joint_names.size());
      last_applied_command_logged_ = latest_command_;
    }
    command_timeout_logged_ = false;
  }

 private:
  void EnsureMappings(const mjModel* m) {
    if (mappings_initialized_ || !m) {
      return;
    }
    for (const JointMapping& mapping : kJointMappings) {
      const int joint_id = mj_name2id(m, mjOBJ_JOINT, mapping.mujoco_joint);
      const int actuator_id =
          mj_name2id(m, mjOBJ_ACTUATOR, mapping.mujoco_actuator);
      if (joint_id >= 0) {
        joint_ids_by_ros_name_[mapping.ros_name] = joint_id;
      }
      if (actuator_id >= 0) {
        actuator_ids_by_ros_name_[mapping.ros_name] = actuator_id;
      }
    }
    mappings_initialized_ = true;
  }

  void PublishJointState(const mjModel* m, const mjData* d) {
    EnsureMappings(m);
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = ToRosTime(d->time);
    msg.name.reserve(kJointMappings.size());
    msg.position.reserve(kJointMappings.size());
    msg.velocity.reserve(kJointMappings.size());

    for (const JointMapping& mapping : kJointMappings) {
      const auto id_iter = joint_ids_by_ros_name_.find(mapping.ros_name);
      if (id_iter == joint_ids_by_ros_name_.end()) {
        continue;
      }
      const int joint_id = id_iter->second;
      msg.name.emplace_back(mapping.ros_name);
      msg.position.push_back(d->qpos[m->jnt_qposadr[joint_id]]);
      msg.velocity.push_back(d->qvel[m->jnt_dofadr[joint_id]]);
    }
    joint_state_pub_->publish(msg);
  }

  void PublishImu(const mjModel* m, const mjData* d) {
    const std::array<double, 4> quat = ReadQuaternionSensor(m, d, "base_quat");
    const std::array<double, 3> gyro = ReadVectorSensor(m, d, "base_gyro");
    const std::array<double, 3> accel = ReadVectorSensor(m, d, "base_accel");

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = ToRosTime(d->time);
    msg.header.frame_id = kBaseFrameId;
    msg.orientation.x = quat[1];
    msg.orientation.y = quat[2];
    msg.orientation.z = quat[3];
    msg.orientation.w = quat[0];
    msg.angular_velocity.x = gyro[0];
    msg.angular_velocity.y = gyro[1];
    msg.angular_velocity.z = gyro[2];
    msg.linear_acceleration.x = accel[0];
    msg.linear_acceleration.y = accel[1];
    msg.linear_acceleration.z = accel[2];
    imu_pub_->publish(msg);
  }

  bool ValidateCommand(const mjModel* m) {
    EnsureMappings(m);
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
      if (actuator_ids_by_ros_name_.find(latest_command_->joint_names[i]) ==
          actuator_ids_by_ros_name_.end()) {
        return false;
      }
    }
    return true;
  }

  bool owns_rclcpp_context_ = false;
  bool mappings_initialized_ = false;
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
  std::unordered_map<std::string, int> joint_ids_by_ros_name_;
  std::unordered_map<std::string, int> actuator_ids_by_ros_name_;
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
