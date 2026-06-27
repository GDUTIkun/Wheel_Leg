#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "wheel_leg_bridge/message_conversions.hpp"
#include "wheel_leg_hw/interface_contract.hpp"
#include "wheel_leg_stm32_bridge/hardware_state_assembler.hpp"
#include "wheel_leg_msgs/msg/control_loop_debug.hpp"
#include "wheel_leg_msgs/msg/joint_command.hpp"
#include "wheel_leg_msgs/msg/rc_status.hpp"
#include "wheel_leg_msgs/msg/stand_control_state.hpp"

namespace wheel_leg_stm32_bridge {
namespace {

constexpr uint8_t kFrameHead0 = 0xA5;
constexpr uint8_t kFrameHead1 = 0x5A;
constexpr uint8_t kFrameTypeCommand = 0x01;
constexpr uint8_t kFrameTypeState = 0x81;
constexpr uint8_t kSafetyStateDisabled = 0;
constexpr uint8_t kSafetyStateEnabled = 1;
constexpr uint8_t kSafetyStateTimeout = 2;
constexpr uint8_t kSafetyStateEstop = 3;
constexpr uint8_t kSafetyStateFault = 4;
constexpr std::size_t kJointCount = 6;
constexpr std::size_t kCommandPayloadSize = 2 + kJointCount * sizeof(float);
constexpr std::size_t kStatePayloadSize =
    4 + 9 * sizeof(float) + kJointCount * 3 * sizeof(float) + 4 + 3 * 4;
constexpr std::size_t kMaxPayloadLen = 160;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesPerRadian = 180.0 / kPi;
// Maps ROS joint torque semantics to the motor-side polarity expected by STM32.
constexpr std::array<float, kJointCount> kCommandEffortSigns = {
    -1.0f,
    -1.0f,
    1.0f,
    1.0f,
    1.0f,
    -1.0f,
};

speed_t ToTermiosBaudRate(int baud_rate) {
  switch (baud_rate) {
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
#ifdef B1000000
    case 1000000:
      return B1000000;
#endif
#ifdef B1500000
    case 1500000:
      return B1500000;
#endif
#ifdef B2000000
    case 2000000:
      return B2000000;
#endif
    default:
      return 0;
  }
}

uint16_t Crc16Ccitt(const uint8_t* data, std::size_t size) {
  uint16_t crc = 0xFFFFu;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000u) != 0u) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

uint16_t ReadU16Le(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t ReadU32Le(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

float ReadF32Le(const uint8_t* data) {
  const uint32_t raw = ReadU32Le(data);
  float value = 0.0f;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

void WriteU16Le(uint8_t* data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFFu);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

void WriteF32Le(uint8_t* data, float value) {
  uint32_t raw = 0u;
  std::memcpy(&raw, &value, sizeof(raw));
  data[0] = static_cast<uint8_t>(raw & 0xFFu);
  data[1] = static_cast<uint8_t>((raw >> 8) & 0xFFu);
  data[2] = static_cast<uint8_t>((raw >> 16) & 0xFFu);
  data[3] = static_cast<uint8_t>((raw >> 24) & 0xFFu);
}

double DegreesToRadians(double degrees) {
  return degrees * kPi / 180.0;
}

double RadiansToDegrees(double radians) {
  return radians * kDegreesPerRadian;
}

double NormalizeLimitAngleDelta(double angle_delta) {
  while (angle_delta > kPi) {
    angle_delta -= 2.0 * kPi;
  }
  while (angle_delta < -kPi) {
    angle_delta += 2.0 * kPi;
  }
  return angle_delta;
}

wheel_leg_common::Quaternion EulerToQuaternion(
    double roll,
    double pitch,
    double yaw) {
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  wheel_leg_common::Quaternion quat;
  quat.w = cr * cp * cy + sr * sp * sy;
  quat.x = sr * cp * cy - cr * sp * sy;
  quat.y = cr * sp * cy + sr * cp * sy;
  quat.z = cr * cp * sy - sr * sp * cy;
  return quat;
}

struct ProtocolStateFrame {
  uint32_t stm_tick_ms = 0;
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  float gyro_x = 0.0f;
  float gyro_y = 0.0f;
  float gyro_z = 0.0f;
  float acc_x = 0.0f;
  float acc_y = 0.0f;
  float acc_z = 0.0f;
  std::array<float, kJointCount> joint_position {};
  std::array<float, kJointCount> joint_velocity {};
  std::array<float, kJointCount> joint_effort {};
  uint8_t online_mask = 0;
  uint8_t safety_state = kSafetyStateDisabled;
  uint8_t last_command_timeout = 0;
  uint32_t comm_rx_error_count = 0;
  uint32_t comm_crc_error_count = 0;
  uint32_t can_error_count = 0;
};

struct BridgeCounters {
  std::atomic<uint64_t> rx_bytes{0};
  std::atomic<uint64_t> rx_frames_ok{0};
  std::atomic<uint64_t> rx_crc_errors{0};
  std::atomic<uint64_t> rx_length_errors{0};
  std::atomic<uint64_t> rx_sync_losses{0};
  std::atomic<uint64_t> rx_unknown_types{0};
  std::atomic<uint64_t> rx_read_errors{0};
  std::atomic<uint64_t> tx_frames_attempted{0};
  std::atomic<uint64_t> tx_frames_sent{0};
  std::atomic<uint64_t> tx_bytes{0};
  std::atomic<uint64_t> tx_write_errors{0};
  std::atomic<uint64_t> tx_partial_writes{0};
};

struct ProtocolCache {
  ProtocolStateFrame state_frame;
  rclcpp::Time received_time{0, 0, RCL_ROS_TIME};
  bool have_state = false;
  double previous_body_time_sec = 0.0;
  bool has_previous_body_time = false;
  HardwareStateAssemblyState hardware_state;
};

struct JointLimitProtectionConfig {
  double effort_threshold_nm = 3.0;
  double left_hip_min_rad = DegreesToRadians(75.0);
  double left_hip_max_rad = DegreesToRadians(200.0);
  double right_hip_min_rad = DegreesToRadians(75.0);
  double right_hip_max_rad = DegreesToRadians(200.0);
  double left_knee_relative_min_rad = DegreesToRadians(-140.0);
  double left_knee_relative_max_rad = DegreesToRadians(-70.0);
  double right_knee_relative_min_rad = DegreesToRadians(-140.0);
  double right_knee_relative_max_rad = DegreesToRadians(-70.0);
};

bool DecodeStatePayload(
    const std::vector<uint8_t>& payload,
    ProtocolStateFrame* frame) {
  if (frame == nullptr || payload.size() != kStatePayloadSize) {
    return false;
  }

  std::size_t offset = 0;
  frame->stm_tick_ms = ReadU32Le(payload.data() + offset);
  offset += 4;
  frame->roll = ReadF32Le(payload.data() + offset);
  offset += 4;
  frame->pitch = ReadF32Le(payload.data() + offset);
  offset += 4;
  frame->yaw = ReadF32Le(payload.data() + offset);
  offset += 4;
  frame->gyro_x = ReadF32Le(payload.data() + offset);
  offset += 4;
  frame->gyro_y = ReadF32Le(payload.data() + offset);
  offset += 4;
  frame->gyro_z = ReadF32Le(payload.data() + offset);
  offset += 4;
  frame->acc_x = ReadF32Le(payload.data() + offset);
  offset += 4;
  frame->acc_y = ReadF32Le(payload.data() + offset);
  offset += 4;
  frame->acc_z = ReadF32Le(payload.data() + offset);
  offset += 4;

  for (std::size_t i = 0; i < kJointCount; ++i) {
    frame->joint_position[i] = ReadF32Le(payload.data() + offset);
    offset += 4;
    frame->joint_velocity[i] = ReadF32Le(payload.data() + offset);
    offset += 4;
    frame->joint_effort[i] = ReadF32Le(payload.data() + offset);
    offset += 4;
  }

  frame->online_mask = payload[offset++];
  frame->safety_state = payload[offset++];
  frame->last_command_timeout = payload[offset++];
  offset += 1;
  frame->comm_rx_error_count = ReadU32Le(payload.data() + offset);
  offset += 4;
  frame->comm_crc_error_count = ReadU32Le(payload.data() + offset);
  offset += 4;
  frame->can_error_count = ReadU32Le(payload.data() + offset);
  return true;
}

std::string SafetyStateToString(uint8_t safety_state) {
  switch (safety_state) {
    case kSafetyStateDisabled:
      return "disabled";
    case kSafetyStateEnabled:
      return "enabled";
    case kSafetyStateTimeout:
      return "timeout";
    case kSafetyStateEstop:
      return "estop";
    case kSafetyStateFault:
      return "fault";
    default:
      return "unknown";
  }
}

}  // namespace

class Stm32BridgeNode : public rclcpp::Node {
 public:
  Stm32BridgeNode() : rclcpp::Node("wheel_leg_stm32_bridge") {
    serial_device_ =
        declare_parameter<std::string>("serial_device", "/dev/ttyAMA4");
    baud_rate_ = declare_parameter<int>("baud_rate", 921600);
    state_timeout_sec_ = declare_parameter<double>("state_timeout_sec", 0.1);
    command_timeout_sec_ =
        declare_parameter<double>("command_timeout_sec", 0.1);
    command_enable_ = declare_parameter<bool>("command_enable", false);
    publish_imu_ = declare_parameter<bool>("publish_imu", true);
    publish_joint_states_ =
        declare_parameter<bool>("publish_joint_states", true);
    hip_velocity_low_pass_alpha_ =
        declare_parameter<double>("hip_velocity_low_pass_alpha", 0.57);
    knee_velocity_low_pass_alpha_ =
        declare_parameter<double>("knee_velocity_low_pass_alpha", 0.62);
    wheel_velocity_low_pass_alpha_ =
        declare_parameter<double>("wheel_velocity_low_pass_alpha", 0.73);
    hardware_state_config_.phi_rate_low_pass_alpha =
        declare_parameter<double>("phi_rate_low_pass_alpha", 0.57);
    hardware_state_config_.length_rate_low_pass_alpha =
        declare_parameter<double>("length_rate_low_pass_alpha", 0.73);
    hardware_state_config_.body_velocity_low_pass_alpha =
        declare_parameter<double>("body_velocity_low_pass_alpha", 0.73);
    status_period_sec_ = declare_parameter<double>("status_period_sec", 0.5);
    joint_limit_protection_.effort_threshold_nm =
        declare_parameter<double>("joint_limit_protection.effort_threshold_nm", 3.0);
    joint_limit_protection_.left_hip_min_rad =
        declare_parameter<double>(
            "joint_limit_protection.left_hip_min_rad", DegreesToRadians(75.0));
    joint_limit_protection_.left_hip_max_rad =
        declare_parameter<double>(
            "joint_limit_protection.left_hip_max_rad", DegreesToRadians(200.0));
    joint_limit_protection_.right_hip_min_rad =
        declare_parameter<double>(
            "joint_limit_protection.right_hip_min_rad", DegreesToRadians(75.0));
    joint_limit_protection_.right_hip_max_rad =
        declare_parameter<double>(
            "joint_limit_protection.right_hip_max_rad", DegreesToRadians(200.0));
    joint_limit_protection_.left_knee_relative_min_rad =
        declare_parameter<double>(
            "joint_limit_protection.left_knee_relative_min_rad",
            DegreesToRadians(-140.0));
    joint_limit_protection_.left_knee_relative_max_rad =
        declare_parameter<double>(
            "joint_limit_protection.left_knee_relative_max_rad",
            DegreesToRadians(-70.0));
    joint_limit_protection_.right_knee_relative_min_rad =
        declare_parameter<double>(
            "joint_limit_protection.right_knee_relative_min_rad",
            DegreesToRadians(-140.0));
    joint_limit_protection_.right_knee_relative_max_rad =
        declare_parameter<double>(
            "joint_limit_protection.right_knee_relative_max_rad",
            DegreesToRadians(-70.0));

    robot_state_pub_ = create_publisher<wheel_leg_msgs::msg::StandControlState>(
        "/robot_state", rclcpp::SystemDefaultsQoS());
    robot_state_raw_pub_ =
        create_publisher<wheel_leg_msgs::msg::StandControlState>(
            "/robot_state_raw", rclcpp::SystemDefaultsQoS());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SystemDefaultsQoS());
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SystemDefaultsQoS());
    joint_state_raw_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states_raw", rclcpp::SystemDefaultsQoS());
    phi_rate_debug_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/stm32_bridge/debug/phi_rate", rclcpp::SystemDefaultsQoS());
    leg_length_rate_debug_pub_ =
        create_publisher<wheel_leg_msgs::msg::ControlLoopDebug>(
            "/stm32_bridge/debug/leg_length_rate",
            rclcpp::SystemDefaultsQoS());
    status_text_pub_ = create_publisher<std_msgs::msg::String>(
        "/stm32_bridge/status_text", 10);
    counters_pub_ = create_publisher<std_msgs::msg::UInt32MultiArray>(
        "/stm32_bridge/counters", 10);
    clear_local_estop_srv_ = create_service<std_srvs::srv::Trigger>(
        "/stm32_bridge/clear_local_estop",
        [this](
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          (void)request;
          HandleClearLocalEstop(response);
        });

    joint_command_sub_ = create_subscription<wheel_leg_msgs::msg::JointCommand>(
        "/joint_command", rclcpp::SystemDefaultsQoS(),
        [this](const wheel_leg_msgs::msg::JointCommand::SharedPtr msg) {
          HandleJointCommand(*msg);
        });
    rc_status_sub_ = create_subscription<wheel_leg_msgs::msg::RcStatus>(
        "/rc/status", rclcpp::SystemDefaultsQoS(),
        [this](const wheel_leg_msgs::msg::RcStatus::SharedPtr msg) {
          bool send_estop_frame = false;
          {
            std::scoped_lock lock(rc_status_mutex_);
            latest_rc_status_ = *msg;
            have_rc_status_ = true;
            send_estop_frame = latest_rc_status_.estop_active;
          }
          if (send_estop_frame) {
            SendCommandFrame({}, true);
          }
        });

    status_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(status_period_sec_)),
        [this]() { PublishBridgeStatus(); });

    if (!OpenSerialPort()) {
      throw std::runtime_error("failed to open STM32 bridge serial port");
    }

    running_.store(true);
    reader_thread_ = std::thread([this]() { ReaderLoop(); });
  }

  ~Stm32BridgeNode() override {
    running_.store(false);
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
    CloseSerialPort();
  }

 private:
  double VelocityFilterAlphaForJoint(std::size_t joint_index) const {
    switch (joint_index) {
      case 0:
      case 3:
        return hip_velocity_low_pass_alpha_;
      case 1:
      case 4:
        return knee_velocity_low_pass_alpha_;
      case 2:
      case 5:
        return wheel_velocity_low_pass_alpha_;
      default:
        return 0.0;
    }
  }

  double FilterJointVelocity(std::size_t joint_index, double raw_velocity) {
    if (joint_index >= filtered_joint_velocities_.size()) {
      return raw_velocity;
    }

    const double alpha = VelocityFilterAlphaForJoint(joint_index);
    if (!has_filtered_joint_velocities_[joint_index]) {
      filtered_joint_velocities_[joint_index] = raw_velocity;
      has_filtered_joint_velocities_[joint_index] = true;
      return raw_velocity;
    }

    filtered_joint_velocities_[joint_index] =
        alpha * filtered_joint_velocities_[joint_index] +
        (1.0 - alpha) * raw_velocity;
    return filtered_joint_velocities_[joint_index];
  }

  bool OpenSerialPort() {
    const speed_t baud = ToTermiosBaudRate(baud_rate_);
    if (baud == 0) {
      RCLCPP_ERROR(
          get_logger(), "Unsupported baud_rate=%d for STM32 bridge", baud_rate_);
      return false;
    }

    serial_fd_ = open(serial_device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(
          get_logger(),
          "Failed to open %s: %s",
          serial_device_.c_str(),
          std::strerror(errno));
      return false;
    }

    termios tty {};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(
          get_logger(),
          "tcgetattr(%s) failed: %s",
          serial_device_.c_str(),
          std::strerror(errno));
      CloseSerialPort();
      return false;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(
          get_logger(),
          "tcsetattr(%s) failed: %s",
          serial_device_.c_str(),
          std::strerror(errno));
      CloseSerialPort();
      return false;
    }

    tcflush(serial_fd_, TCIOFLUSH);
    RCLCPP_INFO(
        get_logger(),
        "Opened STM32 bridge serial device %s at %d 8N1",
        serial_device_.c_str(),
        baud_rate_);
    return true;
  }

  void CloseSerialPort() {
    std::scoped_lock lock(serial_mutex_);
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  void ReaderLoop() {
    std::array<uint8_t, 512> buffer {};
    while (running_.load()) {
      ssize_t bytes_read = 0;
      {
        std::scoped_lock lock(serial_mutex_);
        if (serial_fd_ >= 0) {
          bytes_read = read(serial_fd_, buffer.data(), buffer.size());
        }
      }

      if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        counters_.rx_read_errors++;
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "STM32 bridge read error on %s: %s",
            serial_device_.c_str(),
            std::strerror(errno));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      if (bytes_read == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }

      rx_buffer_.insert(
          rx_buffer_.end(), buffer.begin(), buffer.begin() + bytes_read);
      counters_.rx_bytes += static_cast<uint64_t>(bytes_read);
      ParseFrames();
    }
  }

  void ParseFrames() {
    while (rx_buffer_.size() >= 8u) {
      if (rx_buffer_[0] != kFrameHead0) {
        rx_buffer_.erase(rx_buffer_.begin());
        counters_.rx_sync_losses++;
        continue;
      }
      if (rx_buffer_[1] != kFrameHead1) {
        rx_buffer_.erase(rx_buffer_.begin());
        counters_.rx_sync_losses++;
        continue;
      }

      const std::size_t payload_len = rx_buffer_[3];
      if (payload_len > kMaxPayloadLen) {
        rx_buffer_.erase(rx_buffer_.begin());
        counters_.rx_length_errors++;
        continue;
      }

      const std::size_t frame_len = 2u + 4u + payload_len + 2u;
      if (rx_buffer_.size() < frame_len) {
        return;
      }

      const uint16_t expected_crc =
          Crc16Ccitt(rx_buffer_.data() + 2, 4u + payload_len);
      const uint16_t actual_crc =
          ReadU16Le(rx_buffer_.data() + 6u + payload_len);
      if (expected_crc != actual_crc) {
        rx_buffer_.erase(rx_buffer_.begin());
        counters_.rx_crc_errors++;
        continue;
      }

      const uint8_t frame_type = rx_buffer_[2];
      const uint16_t seq = ReadU16Le(rx_buffer_.data() + 4);
      std::vector<uint8_t> payload(payload_len, 0);
      if (payload_len > 0u) {
        std::copy_n(rx_buffer_.data() + 6, payload_len, payload.begin());
      }
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_len);
      HandleFrame(frame_type, seq, payload);
    }
  }

  void HandleFrame(
      uint8_t frame_type,
      uint16_t seq,
      const std::vector<uint8_t>& payload) {
    (void)seq;
    if (frame_type != kFrameTypeState) {
      counters_.rx_unknown_types++;
      return;
    }

    ProtocolStateFrame decoded;
    if (!DecodeStatePayload(payload, &decoded)) {
      counters_.rx_length_errors++;
      return;
    }
    counters_.rx_frames_ok++;
    PublishDecodedState(decoded);
  }

  void TriggerLocalEstop(const std::string& reason) {
    bool should_send_estop = false;
    {
      std::scoped_lock lock(local_estop_mutex_);
      if (!local_estop_active_) {
        local_estop_active_ = true;
        local_estop_reason_ = reason;
        should_send_estop = true;
      }
    }

    if (should_send_estop) {
      RCLCPP_ERROR(get_logger(), "Local estop triggered: %s", reason.c_str());
      SendCommandFrame({}, true);
    }
  }

  bool GetRcEstopActive() {
    std::scoped_lock lock(rc_status_mutex_);
    return have_rc_status_ && latest_rc_status_.estop_active;
  }

  void HandleClearLocalEstop(
      const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    bool was_active = false;
    std::string previous_reason;
    {
      std::scoped_lock lock(local_estop_mutex_);
      was_active = local_estop_active_;
      previous_reason = local_estop_reason_;
      local_estop_active_ = false;
      local_estop_reason_.clear();
    }

    const bool rc_estop_active = GetRcEstopActive();
    SendCommandFrame({}, rc_estop_active);

    response->success = true;
    if (!was_active) {
      response->message =
          rc_estop_active
              ? "local_estop was already clear; RC estop is still active"
              : "local_estop was already clear";
      return;
    }

    std::ostringstream message;
    message << "Cleared local_estop";
    if (!previous_reason.empty()) {
      message << " (previous_reason=" << previous_reason << ")";
    }
    if (rc_estop_active) {
      message << "; RC estop is still active";
    }
    response->message = message.str();
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  void CheckJointLimitProtection(const ProtocolStateFrame& decoded) {
    const double effort_threshold =
        std::abs(joint_limit_protection_.effort_threshold_nm);
    if (effort_threshold <= 0.0) {
      return;
    }

    const double left_knee_relative =
        NormalizeLimitAngleDelta(
            decoded.joint_position[1] - decoded.joint_position[0]);
    const double right_knee_relative =
        NormalizeLimitAngleDelta(
            decoded.joint_position[4] - decoded.joint_position[3]);
    std::array<float, kJointCount> commanded_efforts {};
    {
      std::scoped_lock lock(command_cache_mutex_);
      commanded_efforts = last_commanded_efforts_;
    }

    const auto trigger_if_needed =
        [this, effort_threshold](
            std::string_view joint_name,
            double observed_position_rad,
            double min_position_rad,
            double max_position_rad,
            double commanded_effort_nm) {
          if (std::abs(commanded_effort_nm) <= effort_threshold) {
            return;
          }
          if (observed_position_rad > min_position_rad &&
              observed_position_rad < max_position_rad) {
            return;
          }

          std::ostringstream reason;
          reason << joint_name << " limit hit: pos_deg="
                 << RadiansToDegrees(observed_position_rad)
                 << " range_deg=[" << RadiansToDegrees(min_position_rad) << ", "
                 << RadiansToDegrees(max_position_rad) << "] cmd_effort_nm="
                 << commanded_effort_nm << " threshold_nm=" << effort_threshold;
          TriggerLocalEstop(reason.str());
        };

    trigger_if_needed(
        "left_hip",
        decoded.joint_position[0],
        joint_limit_protection_.left_hip_min_rad,
        joint_limit_protection_.left_hip_max_rad,
        commanded_efforts[0]);
    trigger_if_needed(
        "right_hip",
        decoded.joint_position[3],
        joint_limit_protection_.right_hip_min_rad,
        joint_limit_protection_.right_hip_max_rad,
        commanded_efforts[3]);
    trigger_if_needed(
        "left_knee_relative",
        left_knee_relative,
        joint_limit_protection_.left_knee_relative_min_rad,
        joint_limit_protection_.left_knee_relative_max_rad,
        commanded_efforts[1]);
    trigger_if_needed(
        "right_knee_relative",
        right_knee_relative,
        joint_limit_protection_.right_knee_relative_min_rad,
        joint_limit_protection_.right_knee_relative_max_rad,
        commanded_efforts[4]);
  }

  void PublishDecodedState(const ProtocolStateFrame& decoded) {
    CheckJointLimitProtection(decoded);

    const rclcpp::Time stamp = now();
    wheel_leg_common::JointStateSample joint_sample;
    wheel_leg_common::JointStateSample filtered_joint_sample;
    const int64_t stamp_ns = stamp.nanoseconds();
    joint_sample.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    joint_sample.stamp.nanosec =
        static_cast<uint32_t>(stamp_ns % 1000000000LL);
    filtered_joint_sample.stamp = joint_sample.stamp;
    joint_sample.joints.reserve(kJointCount);
    filtered_joint_sample.joints.reserve(kJointCount);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      wheel_leg_common::JointSample joint;
      joint.name = std::string(wheel_leg_hw::kCanonicalJointNames[i]);
      joint.position = decoded.joint_position[i];
      joint.velocity = decoded.joint_velocity[i];
      joint.effort = decoded.joint_effort[i];
      joint_sample.joints.push_back(joint);

      wheel_leg_common::JointSample filtered_joint = joint;
      filtered_joint.velocity =
          FilterJointVelocity(i, static_cast<double>(decoded.joint_velocity[i]));
      filtered_joint_sample.joints.push_back(filtered_joint);
    }

    wheel_leg_common::ImuSample imu_sample;
    imu_sample.stamp = joint_sample.stamp;
    imu_sample.frame_id = std::string(wheel_leg_hw::kImuFrame);
    imu_sample.orientation = EulerToQuaternion(
        decoded.roll, decoded.pitch, decoded.yaw);
    imu_sample.angular_velocity.x = decoded.gyro_x;
    imu_sample.angular_velocity.y = decoded.gyro_y;
    imu_sample.angular_velocity.z = decoded.gyro_z;
    imu_sample.linear_acceleration.x = decoded.acc_x;
    imu_sample.linear_acceleration.y = decoded.acc_y;
    imu_sample.linear_acceleration.z = decoded.acc_z;

    const double current_time_sec = stamp.seconds();
    double dt = 0.0;
    {
      std::scoped_lock lock(cache_mutex_);
      if (cache_.has_previous_body_time) {
        dt = current_time_sec - cache_.previous_body_time_sec;
      }
      cache_.previous_body_time_sec = current_time_sec;
      cache_.has_previous_body_time = true;
      cache_.state_frame = decoded;
      cache_.received_time = stamp;
      cache_.have_state = true;
    }

    auto joint_state_raw_msg = wheel_leg_bridge::ToRosJointState(joint_sample);
    joint_state_raw_msg.header.stamp = stamp;
    auto joint_state_msg =
        wheel_leg_bridge::ToRosJointState(filtered_joint_sample);
    joint_state_msg.header.stamp = stamp;
    auto imu_msg = wheel_leg_bridge::ToRosImu(imu_sample);
    imu_msg.header.stamp = stamp;

    wheel_leg_msgs::msg::StandControlState robot_state_msg;
    wheel_leg_msgs::msg::StandControlState robot_state_raw_msg;
    robot_state_msg.header.stamp = stamp;
    robot_state_msg.header.frame_id = std::string(wheel_leg_hw::kImuFrame);
    robot_state_raw_msg.header = robot_state_msg.header;

    HardwareStateAssemblyInput hardware_input;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      hardware_input.joint_position[i] = decoded.joint_position[i];
      hardware_input.joint_velocity[i] = decoded.joint_velocity[i];
    }

    HardwareStateAssemblyOutput hardware_state;
    {
      std::scoped_lock lock(cache_mutex_);
      hardware_state =
          AssembleHardwareState(
              hardware_input, dt, &cache_.hardware_state, hardware_state_config_);
    }
    robot_state_msg.body_distance = hardware_state.body_distance.filtered;
    robot_state_msg.body_velocity = hardware_state.body_velocity.filtered;
    robot_state_msg.body_roll = decoded.roll;
    robot_state_msg.body_roll_rate = decoded.gyro_x;
    robot_state_msg.body_pitch = decoded.pitch;
    robot_state_msg.body_pitch_rate = decoded.gyro_y;
    robot_state_msg.body_yaw_rate = decoded.gyro_z;

    robot_state_msg.left_hip_absolute = hardware_state.left_leg.hip_absolute;
    robot_state_msg.left_calf_absolute = hardware_state.left_leg.calf_absolute;
    robot_state_msg.left_leg_length = hardware_state.left_leg.leg_length;
    robot_state_msg.left_phi = hardware_state.left_leg.phi;
    robot_state_msg.left_phi_rate = hardware_state.left_leg.phi_rate.filtered;
    robot_state_msg.right_hip_absolute = hardware_state.right_leg.hip_absolute;
    robot_state_msg.right_calf_absolute =
        hardware_state.right_leg.calf_absolute;
    robot_state_msg.right_leg_length = hardware_state.right_leg.leg_length;
    robot_state_msg.right_phi = hardware_state.right_leg.phi;
    robot_state_msg.right_phi_rate = hardware_state.right_leg.phi_rate.filtered;

    robot_state_raw_msg.body_distance = hardware_state.body_distance.raw;
    robot_state_raw_msg.body_velocity = hardware_state.body_velocity.raw;
    robot_state_raw_msg.body_roll = decoded.roll;
    robot_state_raw_msg.body_roll_rate = decoded.gyro_x;
    robot_state_raw_msg.body_pitch = decoded.pitch;
    robot_state_raw_msg.body_pitch_rate = decoded.gyro_y;
    robot_state_raw_msg.body_yaw_rate = decoded.gyro_z;
    robot_state_raw_msg.left_hip_absolute = hardware_state.left_leg.hip_absolute;
    robot_state_raw_msg.left_calf_absolute =
        hardware_state.left_leg.calf_absolute;
    robot_state_raw_msg.left_leg_length = hardware_state.left_leg.leg_length;
    robot_state_raw_msg.left_phi = hardware_state.left_leg.phi;
    robot_state_raw_msg.left_phi_rate = hardware_state.left_leg.phi_rate.raw;
    robot_state_raw_msg.right_hip_absolute =
        hardware_state.right_leg.hip_absolute;
    robot_state_raw_msg.right_calf_absolute =
        hardware_state.right_leg.calf_absolute;
    robot_state_raw_msg.right_leg_length = hardware_state.right_leg.leg_length;
    robot_state_raw_msg.right_phi = hardware_state.right_leg.phi;
    robot_state_raw_msg.right_phi_rate = hardware_state.right_leg.phi_rate.raw;

    robot_state_pub_->publish(robot_state_msg);
    robot_state_raw_pub_->publish(robot_state_raw_msg);
    if (publish_imu_) {
      imu_pub_->publish(imu_msg);
    }
    if (publish_joint_states_) {
      joint_state_raw_pub_->publish(joint_state_raw_msg);
      joint_state_pub_->publish(joint_state_msg);
    }

    wheel_leg_msgs::msg::ControlLoopDebug phi_rate_msg;
    phi_rate_msg.header.stamp = stamp;
    phi_rate_msg.loop_name = "stm32_bridge_phi_rate";
    phi_rate_msg.ref_primary = hardware_state.left_leg.phi_rate.raw;
    phi_rate_msg.now_primary = hardware_state.left_leg.phi_rate.filtered;
    phi_rate_msg.ref_secondary = hardware_state.right_leg.phi_rate.raw;
    phi_rate_msg.now_secondary = hardware_state.right_leg.phi_rate.filtered;
    phi_rate_debug_pub_->publish(phi_rate_msg);

    wheel_leg_msgs::msg::ControlLoopDebug leg_length_rate_msg;
    leg_length_rate_msg.header.stamp = stamp;
    leg_length_rate_msg.loop_name = "stm32_bridge_leg_length_rate";
    leg_length_rate_msg.ref_primary = hardware_state.left_leg.length_rate.raw;
    leg_length_rate_msg.now_primary =
        hardware_state.left_leg.length_rate.filtered;
    leg_length_rate_msg.ref_secondary =
        hardware_state.right_leg.length_rate.raw;
    leg_length_rate_msg.now_secondary =
        hardware_state.right_leg.length_rate.filtered;
    leg_length_rate_debug_pub_->publish(leg_length_rate_msg);
  }

  void HandleJointCommand(const wheel_leg_msgs::msg::JointCommand& msg) {
    std::array<float, kJointCount> canonical_efforts {};
    for (std::size_t index = 0; index < msg.joint_names.size(); ++index) {
      const std::string& joint_name = msg.joint_names[index];
      auto it = std::find_if(
          wheel_leg_hw::kCanonicalJointNames.begin(),
          wheel_leg_hw::kCanonicalJointNames.end(),
          [&joint_name](std::string_view known_name) {
            return joint_name == known_name;
          });
      if (it == wheel_leg_hw::kCanonicalJointNames.end()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Ignoring /joint_command entry for unknown joint '%s'",
            joint_name.c_str());
        continue;
      }

      const std::size_t joint_index =
          static_cast<std::size_t>(std::distance(
              wheel_leg_hw::kCanonicalJointNames.begin(), it));
      canonical_efforts[joint_index] =
          static_cast<float>(index < msg.efforts.size() ? msg.efforts[index] : 0.0);
    }

    bool estop_active = false;
    {
      std::scoped_lock lock(rc_status_mutex_);
      if (have_rc_status_) {
        estop_active = latest_rc_status_.estop_active;
      }
    }
    {
      std::scoped_lock lock(local_estop_mutex_);
      estop_active = estop_active || local_estop_active_;
    }
    {
      std::scoped_lock lock(command_cache_mutex_);
      last_commanded_efforts_ = canonical_efforts;
    }

    SendCommandFrame(canonical_efforts, estop_active);
  }

  void SendCommandFrame(
      const std::array<float, kJointCount>& efforts,
      bool estop_active) {
    std::array<uint8_t, kCommandPayloadSize> payload {};
    const bool command_enable = command_enable_ && !estop_active;
    payload[0] = command_enable ? 1u : 0u;
    payload[1] = estop_active ? 1u : 0u;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const float effort =
          command_enable ? efforts[i] * kCommandEffortSigns[i] : 0.0f;
      WriteF32Le(payload.data() + 2u + i * sizeof(float), effort);
    }

    std::vector<uint8_t> frame(2u + 4u + payload.size() + 2u, 0);
    frame[0] = kFrameHead0;
    frame[1] = kFrameHead1;
    frame[2] = kFrameTypeCommand;
    frame[3] = static_cast<uint8_t>(payload.size());
    WriteU16Le(frame.data() + 4, tx_seq_);
    std::copy(payload.begin(), payload.end(), frame.begin() + 6);
    const uint16_t crc = Crc16Ccitt(frame.data() + 2, 4u + payload.size());
    WriteU16Le(frame.data() + 6u + payload.size(), crc);

    counters_.tx_frames_attempted++;
    ssize_t bytes_written = -1;
    {
      std::scoped_lock lock(serial_mutex_);
      if (serial_fd_ >= 0) {
        bytes_written = write(serial_fd_, frame.data(), frame.size());
      }
    }
    if (bytes_written < 0) {
      counters_.tx_write_errors++;
      RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "STM32 bridge write error on %s: %s",
          serial_device_.c_str(),
          std::strerror(errno));
      return;
    }
    if (static_cast<std::size_t>(bytes_written) != frame.size()) {
      counters_.tx_partial_writes++;
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Partial STM32 frame write: wanted=%zu wrote=%zd",
          frame.size(),
          bytes_written);
      return;
    }

    counters_.tx_frames_sent++;
    counters_.tx_bytes += static_cast<uint64_t>(bytes_written);
    tx_seq_ = static_cast<uint16_t>(tx_seq_ + 1u);
    last_command_time_ = now();
  }

  void PublishBridgeStatus() {
    std_msgs::msg::UInt32MultiArray counters_msg;
    counters_msg.data = {
        static_cast<uint32_t>(counters_.rx_frames_ok.load()),
        static_cast<uint32_t>(counters_.rx_crc_errors.load()),
        static_cast<uint32_t>(counters_.rx_length_errors.load()),
        static_cast<uint32_t>(counters_.rx_sync_losses.load()),
        static_cast<uint32_t>(counters_.tx_frames_sent.load()),
        static_cast<uint32_t>(counters_.tx_write_errors.load()),
        static_cast<uint32_t>(counters_.tx_partial_writes.load()),
    };
    counters_pub_->publish(counters_msg);

    ProtocolStateFrame latest_state;
    rclcpp::Time received_time{0, 0, RCL_ROS_TIME};
    bool have_state = false;
    {
      std::scoped_lock lock(cache_mutex_);
      latest_state = cache_.state_frame;
      received_time = cache_.received_time;
      have_state = cache_.have_state;
    }

    const bool state_stale =
        have_state &&
        ((now() - received_time).seconds() > state_timeout_sec_);
    const bool command_stale =
        last_command_time_.nanoseconds() != 0 &&
        ((now() - last_command_time_).seconds() > command_timeout_sec_);

    bool estop_active = false;
    {
      std::scoped_lock lock(rc_status_mutex_);
      estop_active = have_rc_status_ && latest_rc_status_.estop_active;
    }
    std::string local_estop_reason;
    bool local_estop_active = false;
    {
      std::scoped_lock lock(local_estop_mutex_);
      local_estop_active = local_estop_active_;
      local_estop_reason = local_estop_reason_;
    }
    estop_active = estop_active || local_estop_active;

    std_msgs::msg::String text_msg;
    std::ostringstream status;
    status << "state=" << (have_state ? "ok" : "missing")
           << " state_stale=" << (state_stale ? "true" : "false")
           << " command_stale=" << (command_stale ? "true" : "false")
           << " command_enable=" << (command_enable_ ? "true" : "false")
           << " safety=" << SafetyStateToString(latest_state.safety_state)
           << " estop=" << (estop_active ? "true" : "false")
           << " local_estop=" << (local_estop_active ? "true" : "false");
    if (local_estop_active && !local_estop_reason.empty()) {
      status << " local_estop_reason=" << local_estop_reason;
    }
    text_msg.data = status.str();
    status_text_pub_->publish(text_msg);
  }

  std::string serial_device_;
  int baud_rate_ = 0;
  double state_timeout_sec_ = 0.0;
  double command_timeout_sec_ = 0.0;
  bool command_enable_ = false;
  bool publish_imu_ = true;
  bool publish_joint_states_ = true;
  double hip_velocity_low_pass_alpha_ = 0.57;
  double knee_velocity_low_pass_alpha_ = 0.62;
  double wheel_velocity_low_pass_alpha_ = 0.73;
  double status_period_sec_ = 0.0;
  JointLimitProtectionConfig joint_limit_protection_;
  HardwareStateAssemblerConfig hardware_state_config_;
  int serial_fd_ = -1;
  std::atomic<bool> running_{false};
  std::thread reader_thread_;
  std::mutex serial_mutex_;
  std::mutex cache_mutex_;
  std::mutex command_cache_mutex_;
  std::mutex local_estop_mutex_;
  std::vector<uint8_t> rx_buffer_;
  ProtocolCache cache_;
  BridgeCounters counters_;
  uint16_t tx_seq_ = 0;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  std::array<float, kJointCount> last_commanded_efforts_ {};
  std::array<double, kJointCount> filtered_joint_velocities_ {};
  std::array<bool, kJointCount> has_filtered_joint_velocities_ {};
  std::mutex rc_status_mutex_;
  wheel_leg_msgs::msg::RcStatus latest_rc_status_;
  bool have_rc_status_ = false;
  bool local_estop_active_ = false;
  std::string local_estop_reason_;

  rclcpp::Publisher<wheel_leg_msgs::msg::StandControlState>::SharedPtr
      robot_state_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::StandControlState>::SharedPtr
      robot_state_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_raw_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      phi_rate_debug_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::ControlLoopDebug>::SharedPtr
      leg_length_rate_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_text_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr counters_pub_;
  rclcpp::Subscription<wheel_leg_msgs::msg::JointCommand>::SharedPtr
      joint_command_sub_;
  rclcpp::Subscription<wheel_leg_msgs::msg::RcStatus>::SharedPtr rc_status_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_local_estop_srv_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace wheel_leg_stm32_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<wheel_leg_stm32_bridge::Stm32BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
