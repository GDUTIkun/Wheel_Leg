#include "ros2_bridge.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "sensor.h"
#include <wheel_leg_sim/command_application.hpp>
#include <wheel_leg_sim/joint_mappings.hpp>
#include <wheel_leg_sim/mapping_utils.hpp>
#include <wheel_leg_sim/state_builders.hpp>

#ifdef WHEEL_LEG_ENABLE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wheel_leg_bridge/message_conversions.hpp>
#include <wheel_leg_control/state_message_conversions.hpp>
#include <wheel_leg_msgs/msg/joint_command.hpp>
#include <wheel_leg_msgs/msg/stand_control_state.hpp>
#include <wheel_leg_sim/control_state_bridge.hpp>
#endif

namespace wheel_leg {
namespace {

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

bool NearlyEqual(double a, double b, double tolerance = 1e-9) {
  return std::fabs(a - b) <= tolerance;
}

using ApplyCommandResult = wheel_leg_sim::ApplyCommandResult;

wheel_leg_common::JointStateSample SampleJointState(
    const mjModel* m, const mjData* d) {
  wheel_leg_common::JointStateSample sample;
  if (!m || !d) {
    return sample;
  }

  sample.stamp = wheel_leg_sim::SimTimeToCommonTime(d->time);
  sample.joints.reserve(wheel_leg_sim::kJointMappings.size());

  for (const auto& mapping : wheel_leg_sim::kJointMappings) {
    const int joint_id = mj_name2id(m, mjOBJ_JOINT, mapping.mujoco_joint);
    if (joint_id < 0) {
      continue;
    }

    wheel_leg_sim::AppendJointSample(
        &sample,
        mapping.ros_name,
        d->qpos[m->jnt_qposadr[joint_id]],
        d->qvel[m->jnt_dofadr[joint_id]]);
  }

  return sample;
}

wheel_leg_common::ImuSample SampleImu(const mjModel* m, const mjData* d) {
  wheel_leg_common::ImuSample sample;
  if (!m || !d) {
    return sample;
  }

  const std::array<double, 4> quat = ReadQuaternionSensor(m, d, "base_quat");
  const std::array<double, 3> gyro = ReadVectorSensor(m, d, "base_gyro");
  const std::array<double, 3> accel = ReadVectorSensor(m, d, "base_accel");

  return wheel_leg_sim::BuildImuSample(
      wheel_leg_sim::SimTimeToCommonTime(d->time),
      "base_link",
      quat[0], quat[1], quat[2], quat[3],
      gyro[0], gyro[1], gyro[2],
      accel[0], accel[1], accel[2]);
}

ApplyCommandResult ApplyControlCommand(
    const mjModel* m, mjData* d,
    const wheel_leg_common::ControlCommand& command) {
  if (!m || !d) {
    return ApplyCommandResult();
  }

  std::vector<wheel_leg_sim::PreparedActuatorCommand> prepared_commands;
  const ApplyCommandResult result = wheel_leg_sim::PrepareActuatorCommands(
      command,
      [m](std::string_view actuator_name)
          -> std::optional<wheel_leg_sim::ActuatorControlRange> {
        const int actuator_id =
            mj_name2id(m, mjOBJ_ACTUATOR, std::string(actuator_name).c_str());
        if (actuator_id < 0) {
          return std::nullopt;
        }

        wheel_leg_sim::ActuatorControlRange range;
        range.limited = m->actuator_ctrllimited[actuator_id];
        if (range.limited) {
          range.min_effort = m->actuator_ctrlrange[2 * actuator_id];
          range.max_effort = m->actuator_ctrlrange[2 * actuator_id + 1];
        }
        return range;
      },
      &prepared_commands);
  if (!result.accepted) {
    return result;
  }

  for (const auto& prepared_command : prepared_commands) {
    const int actuator_id =
        mj_name2id(m, mjOBJ_ACTUATOR, prepared_command.actuator_name.c_str());
    if (actuator_id < 0) {
      ApplyCommandResult failed_result = result;
      failed_result.accepted = false;
      failed_result.rejected_joint_name = prepared_command.actuator_name;
      return failed_result;
    }
    d->ctrl[actuator_id] = prepared_command.effort;
  }

  return result;
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
    robot_state_pub_ =
        node_->create_publisher<wheel_leg_msgs::msg::StandControlState>(
            "/robot_state", rclcpp::SystemDefaultsQoS());
    command_sub_ =
        node_->create_subscription<wheel_leg_msgs::msg::JointCommand>(
            "/joint_command", rclcpp::SystemDefaultsQoS(),
            [this](wheel_leg_msgs::msg::JointCommand::SharedPtr msg) {
              latest_command_ = std::move(msg);
              pending_command_update_ = true;
            });

    std::cout << "ROS2 MuJoCo bridge ready: node=" << kNodeName
              << ", topics=/joint_states,/imu,/robot_state,/joint_command"
              << ", enable_ros_command="
              << (enable_ros_command_ ? "true" : "false") << std::endl;
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
    PublishRobotState(m, d);
    PublishJointState(m, d);
    PublishImu(m, d);
  }

  void ApplyCommand(const mjModel* m, mjData* d) {
    if (!node_ || !rclcpp::ok() || !m || !d || !HasActiveCommand(d->time)) {
      return;
    }

    wheel_leg_common::ControlCommand command =
        wheel_leg_bridge::FromRosJointCommand(*latest_command_);
    command.stamp = ToCommonTime(d->time);

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

  bool HasActiveCommand(double sim_time) {
    if (!enable_ros_command_ || !has_command_) {
      return false;
    }
    if (sim_time - latest_command_sim_time_ > kCommandTimeoutSec) {
      if (!command_timeout_logged_) {
        RCLCPP_WARN(node_->get_logger(),
                    "/joint_command timed out after %.3f s; actuator writes are suspended",
                    kCommandTimeoutSec);
        command_timeout_logged_ = true;
      }
      return false;
    }
    if (!ValidateCommand()) {
      if (latest_command_ != last_invalid_command_logged_) {
        RCLCPP_WARN(node_->get_logger(),
                    "Rejected invalid /joint_command; no actuator was written");
        last_invalid_command_logged_ = latest_command_;
      }
      return false;
    }
    return true;
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

  void PublishRobotState(const mjModel* m, const mjData* d) {
    const RobotSensorData sensor_data = AssembleSensorData(m, d);
    const auto control_state = wheel_leg_sim::BuildStandControlState(sensor_data);
    if (!logged_robot_state_sample_) {
      logged_robot_state_sample_ = true;
      RCLCPP_INFO(
          node_->get_logger(),
          "Robot state sample: body=(dist=%.6f, vel=%.6f, roll=%.6f, roll_rate=%.6f, pitch=%.6f, pitch_rate=%.6f, yaw_rate=%.6f), "
          "left=(hip=%.6f, calf=%.6f, len=%.6f, phi=%.6f, phi_rate=%.6f), "
          "right=(hip=%.6f, calf=%.6f, len=%.6f, phi=%.6f, phi_rate=%.6f)",
          control_state.body.distance,
          control_state.body.velocity,
          control_state.body.roll,
          control_state.body.roll_rate,
          control_state.body.pitch,
          control_state.body.pitch_rate,
          control_state.body.yaw_rate,
          control_state.left_leg.hip_absolute,
          control_state.left_leg.calf_absolute,
          control_state.left_leg.leg_length,
          control_state.left_leg.phi,
          control_state.left_leg.phi_rate,
          control_state.right_leg.hip_absolute,
          control_state.right_leg.calf_absolute,
          control_state.right_leg.leg_length,
          control_state.right_leg.phi,
          control_state.right_leg.phi_rate);
    }
    const auto stamp = ToCommonTime(d->time);
    const auto ros_state =
        wheel_leg_control::ToRosStandControlState(control_state, stamp);
    VerifyRobotStateRoundTrip(control_state, ros_state);
    robot_state_pub_->publish(ros_state);
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

  void VerifyRobotStateRoundTrip(
      const wheel_leg_control::StandControlState& expected,
      const wheel_leg_msgs::msg::StandControlState& ros_state) {
    if (robot_state_round_trip_checked_) {
      return;
    }
    robot_state_round_trip_checked_ = true;

    const auto round_tripped =
        wheel_leg_control::FromRosStandControlState(ros_state);
    std::ostringstream mismatch;
    const auto append_if_mismatch =
        [&mismatch](const char* label, double lhs, double rhs) {
          if (!NearlyEqual(lhs, rhs)) {
            if (mismatch.tellp() > 0) {
              mismatch << ", ";
            }
            mismatch << label << ": " << lhs << " != " << rhs;
          }
        };

    append_if_mismatch("body.distance", expected.body.distance,
                       round_tripped.body.distance);
    append_if_mismatch("body.velocity", expected.body.velocity,
                       round_tripped.body.velocity);
    append_if_mismatch("body.roll", expected.body.roll,
                       round_tripped.body.roll);
    append_if_mismatch("body.roll_rate", expected.body.roll_rate,
                       round_tripped.body.roll_rate);
    append_if_mismatch("body.pitch", expected.body.pitch,
                       round_tripped.body.pitch);
    append_if_mismatch("body.pitch_rate", expected.body.pitch_rate,
                       round_tripped.body.pitch_rate);
    append_if_mismatch("body.yaw_rate", expected.body.yaw_rate,
                       round_tripped.body.yaw_rate);
    append_if_mismatch("left.hip_absolute", expected.left_leg.hip_absolute,
                       round_tripped.left_leg.hip_absolute);
    append_if_mismatch("left.calf_absolute", expected.left_leg.calf_absolute,
                       round_tripped.left_leg.calf_absolute);
    append_if_mismatch("left.leg_length", expected.left_leg.leg_length,
                       round_tripped.left_leg.leg_length);
    append_if_mismatch("left.phi", expected.left_leg.phi,
                       round_tripped.left_leg.phi);
    append_if_mismatch("left.phi_rate", expected.left_leg.phi_rate,
                       round_tripped.left_leg.phi_rate);
    append_if_mismatch("right.hip_absolute", expected.right_leg.hip_absolute,
                       round_tripped.right_leg.hip_absolute);
    append_if_mismatch("right.calf_absolute", expected.right_leg.calf_absolute,
                       round_tripped.right_leg.calf_absolute);
    append_if_mismatch("right.leg_length", expected.right_leg.leg_length,
                       round_tripped.right_leg.leg_length);
    append_if_mismatch("right.phi", expected.right_leg.phi,
                       round_tripped.right_leg.phi);
    append_if_mismatch("right.phi_rate", expected.right_leg.phi_rate,
                       round_tripped.right_leg.phi_rate);

    if (mismatch.tellp() > 0) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Robot state round-trip mismatch detected: %s",
                   mismatch.str().c_str());
      return;
    }

    RCLCPP_INFO(node_->get_logger(),
                "Robot state round-trip verified for /robot_state publish path.");
  }

  bool owns_rclcpp_context_ = false;
  bool enable_ros_command_ = false;
  bool pending_command_update_ = false;
  bool has_command_ = false;
  bool command_timeout_logged_ = false;
  bool logged_robot_state_sample_ = false;
  bool robot_state_round_trip_checked_ = false;
  double latest_command_sim_time_ = 0.0;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::StandControlState>::SharedPtr
      robot_state_pub_;
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

bool HasActiveRosCommandControl(const mjData* d) {
#ifdef WHEEL_LEG_ENABLE_ROS2
  if (g_bridge && d) {
    return g_bridge->HasActiveCommand(d->time);
  }
#else
  (void)d;
#endif
  return false;
}

bool StepRosCommandControl(const mjModel* m, mjData* d) {
#ifdef WHEEL_LEG_ENABLE_ROS2
  static bool last_ros_takeover_active = false;
  SpinRos2Bridge(m, d);
  const bool ros_takeover_active = HasActiveRosCommandControl(d);
  if (ros_takeover_active != last_ros_takeover_active) {
    if (ros_takeover_active) {
      std::cout << "ROS command takeover active: bypassing legacy stand control."
                << std::endl;
    } else {
      std::cout << "ROS command takeover released: returning to legacy stand control."
                << std::endl;
    }
    last_ros_takeover_active = ros_takeover_active;
  }
  if (ros_takeover_active) {
    ApplyRos2Command(m, d);
  }
  return ros_takeover_active;
#else
  (void)m;
  (void)d;
  return false;
#endif
}

}  // namespace wheel_leg
