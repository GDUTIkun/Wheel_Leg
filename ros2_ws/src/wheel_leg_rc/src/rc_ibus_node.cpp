#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <wheel_leg_msgs/msg/body_command.hpp>
#include <wheel_leg_msgs/msg/rc_channels_raw.hpp>
#include <wheel_leg_msgs/msg/rc_status.hpp>

namespace wheel_leg_rc {
namespace {

constexpr const char* kDefaultSerialDevice = "/dev/ttyAMA3";
constexpr int kDefaultBaudRate = 115200;
constexpr int kIbusFrameLength = 32;
constexpr int kIbusChannelCount = 14;
constexpr uint8_t kIbusLengthByte = 0x20;
constexpr uint8_t kIbusCommandByte = 0x40;

speed_t ToTermiosBaudRate(int baud_rate) {
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      return 0;
  }
}

struct ChannelMapping {
  int channel = 1;
  double min = 1000.0;
  double center = 1500.0;
  double max = 2000.0;
  bool reverse = false;
  double deadzone = 0.05;
  double scale = 1.0;
};

struct IbusFrame {
  std::array<uint16_t, kIbusChannelCount> channels {};
};

struct Counters {
  uint32_t valid_frame_count = 0;
  uint32_t checksum_error_count = 0;
  uint32_t frame_sync_loss_count = 0;
  uint32_t serial_read_error_count = 0;
  uint32_t serial_open_failure_count = 0;
};

double ApplyDeadzone(double value, double deadzone) {
  const double magnitude = std::abs(value);
  if (magnitude <= deadzone) {
    return 0.0;
  }

  const double scaled = (magnitude - deadzone) / (1.0 - deadzone);
  return std::copysign(std::clamp(scaled, 0.0, 1.0), value);
}

}  // namespace

class RcIbusNode : public rclcpp::Node {
 public:
  RcIbusNode()
      : rclcpp::Node("rc_ibus_node") {
    serial_device_ =
        declare_parameter<std::string>("serial_device", kDefaultSerialDevice);
    baud_rate_ = declare_parameter<int>("baud_rate", kDefaultBaudRate);
    read_period_ms_ = declare_parameter<int>("read_period_ms", 5);
    status_period_ms_ = declare_parameter<int>("status_period_ms", 100);
    frame_timeout_ms_ = declare_parameter<int>("frame_timeout_ms", 100);
    body_height_scale_ = declare_parameter<double>("body_height_scale", 0.15);
    yaw_rate_assist_scale_ =
        declare_parameter<double>("yaw_rate_assist_scale", 1.0);

    linear_x_mapping_ = LoadChannelMapping("linear_x", 2, 1.0);
    angular_z_mapping_ = LoadChannelMapping("angular_z", 4, 1.0);
    body_height_mapping_ = LoadChannelMapping("body_height", 3, 1.0);
    yaw_rate_assist_mapping_ = LoadChannelMapping("yaw_rate_assist", 1, 1.0);

    mode_channel_ = declare_parameter<int>("control_mode.channel", 6);
    mode_low_threshold_ =
        declare_parameter<double>("control_mode.low_threshold", 1333.0);
    mode_high_threshold_ =
        declare_parameter<double>("control_mode.high_threshold", 1666.0);
    mode_low_label_ =
        declare_parameter<std::string>("control_mode.low_label", "stand");
    mode_mid_label_ =
        declare_parameter<std::string>("control_mode.mid_label", "stand");
    mode_high_label_ =
        declare_parameter<std::string>("control_mode.high_label", "velocity");
    estop_channel_ = declare_parameter<int>("estop.channel", 7);
    estop_threshold_ = declare_parameter<double>("estop.threshold", 1700.0);
    estop_active_below_ =
        declare_parameter<bool>("estop.active_below", false);
    estop_mode_label_ =
        declare_parameter<std::string>("estop.mode_label", "disabled");
    safe_control_mode_ =
        declare_parameter<std::string>("safe_control_mode", "stand");

    raw_pub_ = create_publisher<wheel_leg_msgs::msg::RcChannelsRaw>(
        "/rc/channels_raw", 10);
    status_pub_ =
        create_publisher<wheel_leg_msgs::msg::RcStatus>("/rc/status", 10);
    cmd_vel_pub_ =
        create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    control_mode_pub_ =
        create_publisher<std_msgs::msg::String>("/control_mode", 10);
    body_cmd_pub_ = create_publisher<wheel_leg_msgs::msg::BodyCommand>(
        "/body_cmd", 10);

    if (!OpenSerialPort()) {
      RCLCPP_WARN(
          get_logger(),
          "rc_ibus_node will retry opening %s until the port becomes available",
          serial_device_.c_str());
    }

    read_timer_ = create_wall_timer(
        std::chrono::milliseconds(read_period_ms_),
        [this]() { PollSerialPort(); });
    status_timer_ = create_wall_timer(
        std::chrono::milliseconds(status_period_ms_),
        [this]() { PublishStatusAndFailsafe(); });
  }

  ~RcIbusNode() override {
    CloseSerialPort();
  }

 private:
  ChannelMapping LoadChannelMapping(
      const std::string& prefix,
      int default_channel,
      double default_scale) {
    ChannelMapping mapping;
    const bool default_reverse = (prefix == "angular_z");
    mapping.channel =
        declare_parameter<int>(prefix + ".channel", default_channel);
    mapping.min = declare_parameter<double>(prefix + ".min", 1000.0);
    mapping.center = declare_parameter<double>(prefix + ".center", 1500.0);
    mapping.max = declare_parameter<double>(prefix + ".max", 2000.0);
    mapping.reverse =
        declare_parameter<bool>(prefix + ".reverse", default_reverse);
    mapping.deadzone =
        declare_parameter<double>(prefix + ".deadzone", 0.05);
    mapping.scale = declare_parameter<double>(prefix + ".scale", default_scale);
    return mapping;
  }

  bool OpenSerialPort() {
    CloseSerialPort();

    const speed_t baud = ToTermiosBaudRate(baud_rate_);
    if (baud == 0) {
      RCLCPP_ERROR(
          get_logger(),
          "Unsupported baud_rate=%d; supported values include 9600, 19200, 38400, 57600, 115200",
          baud_rate_);
      return false;
    }

    serial_fd_ = open(serial_device_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      ++counters_.serial_open_failure_count;
      serial_online_ = false;
      RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Failed to open %s: %s",
          serial_device_.c_str(),
          std::strerror(errno));
      return false;
    }

    termios tty {};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      ++counters_.serial_open_failure_count;
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
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      ++counters_.serial_open_failure_count;
      RCLCPP_ERROR(
          get_logger(),
          "tcsetattr(%s) failed: %s",
          serial_device_.c_str(),
          std::strerror(errno));
      CloseSerialPort();
      return false;
    }

    tcflush(serial_fd_, TCIFLUSH);
    serial_online_ = true;
    rx_buffer_.clear();
    RCLCPP_INFO(
        get_logger(),
        "Opened %s with %d 8N1 for iBUS reception",
        serial_device_.c_str(),
        baud_rate_);
    return true;
  }

  void CloseSerialPort() {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
    serial_online_ = false;
  }

  void PollSerialPort() {
    if (serial_fd_ < 0) {
      OpenSerialPort();
      return;
    }

    std::array<uint8_t, 512> buffer {};
    const ssize_t bytes_read = read(serial_fd_, buffer.data(), buffer.size());
    if (bytes_read < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }

      ++counters_.serial_read_error_count;
      serial_online_ = false;
      RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Read error on %s: %s",
          serial_device_.c_str(),
          std::strerror(errno));
      CloseSerialPort();
      return;
    }

    if (bytes_read == 0) {
      return;
    }

    serial_online_ = true;
    rx_buffer_.insert(
        rx_buffer_.end(),
        buffer.begin(),
        buffer.begin() + bytes_read);
    ParseFrames();
  }

  void ParseFrames() {
    while (rx_buffer_.size() >= kIbusFrameLength) {
      const auto header_it = std::search(
          rx_buffer_.begin(),
          rx_buffer_.end(),
          kIbusHeader_.begin(),
          kIbusHeader_.end());

      if (header_it == rx_buffer_.end()) {
        counters_.frame_sync_loss_count +=
            static_cast<uint32_t>(rx_buffer_.size() - 1U);
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - 1);
        return;
      }

      if (header_it != rx_buffer_.begin()) {
        counters_.frame_sync_loss_count += static_cast<uint32_t>(
            std::distance(rx_buffer_.begin(), header_it));
        rx_buffer_.erase(rx_buffer_.begin(), header_it);
      }

      if (rx_buffer_.size() < kIbusFrameLength) {
        return;
      }

      const uint16_t expected_checksum = ComputeChecksum(rx_buffer_.data());
      const uint16_t actual_checksum =
          static_cast<uint16_t>(rx_buffer_[30]) |
          (static_cast<uint16_t>(rx_buffer_[31]) << 8U);
      if (expected_checksum != actual_checksum) {
        ++counters_.checksum_error_count;
        ++counters_.frame_sync_loss_count;
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }

      IbusFrame frame;
      for (int channel_index = 0; channel_index < kIbusChannelCount;
           ++channel_index) {
        const std::size_t byte_index = 2 + channel_index * 2;
        frame.channels[channel_index] =
            static_cast<uint16_t>(rx_buffer_[byte_index]) |
            (static_cast<uint16_t>(rx_buffer_[byte_index + 1]) << 8U);
      }

      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + kIbusFrameLength);
      HandleValidFrame(frame);
    }
  }

  uint16_t ComputeChecksum(const uint8_t* frame) const {
    uint32_t sum = 0;
    for (int index = 0; index < 30; ++index) {
      sum += frame[index];
    }
    return static_cast<uint16_t>(0xFFFFU - (sum & 0xFFFFU));
  }

  void HandleValidFrame(const IbusFrame& frame) {
    latest_channels_ = frame.channels;
    have_valid_frame_ = true;
    ++counters_.valid_frame_count;
    last_valid_frame_time_ = now();
    PublishRawChannels();
    PublishMappedCommands(false);
  }

  void PublishRawChannels() {
    wheel_leg_msgs::msg::RcChannelsRaw message;
    message.header.stamp = now();
    message.protocol = "iBUS";
    message.frame_valid = true;
    message.channels.assign(
        latest_channels_.begin(),
        latest_channels_.end());
    raw_pub_->publish(message);
  }

  double GetMappedAxis(const ChannelMapping& mapping) const {
    if (!have_valid_frame_) {
      return 0.0;
    }

    const int channel_index = mapping.channel - 1;
    if (channel_index < 0 || channel_index >= kIbusChannelCount) {
      return 0.0;
    }

    const double raw = static_cast<double>(latest_channels_[channel_index]);
    double normalized = 0.0;
    if (raw >= mapping.center) {
      const double denominator = std::max(1.0, mapping.max - mapping.center);
      normalized = (raw - mapping.center) / denominator;
    } else {
      const double denominator = std::max(1.0, mapping.center - mapping.min);
      normalized = (raw - mapping.center) / denominator;
    }

    normalized = std::clamp(normalized, -1.0, 1.0);
    normalized = ApplyDeadzone(normalized, std::clamp(mapping.deadzone, 0.0, 0.99));
    if (mapping.reverse) {
      normalized = -normalized;
    }
    return normalized * mapping.scale;
  }

  std::string GetModeLabel() const {
    if (!have_valid_frame_) {
      return safe_control_mode_;
    }

    const int channel_index = mode_channel_ - 1;
    if (channel_index < 0 || channel_index >= kIbusChannelCount) {
      return safe_control_mode_;
    }

    const double raw = static_cast<double>(latest_channels_[channel_index]);
    if (raw < mode_low_threshold_) {
      return mode_low_label_;
    }
    if (raw < mode_high_threshold_) {
      return mode_mid_label_;
    }
    return mode_high_label_;
  }

  bool IsEstopActive() const {
    if (!have_valid_frame_) {
      return false;
    }

    const int channel_index = estop_channel_ - 1;
    if (channel_index < 0 || channel_index >= kIbusChannelCount) {
      return false;
    }

    const double raw = static_cast<double>(latest_channels_[channel_index]);
    return estop_active_below_ ? (raw < estop_threshold_)
                               : (raw > estop_threshold_);
  }

  bool IsFrameTimedOut() const {
    if (!have_valid_frame_) {
      return true;
    }

    const auto elapsed = now() - last_valid_frame_time_;
    return elapsed >
           rclcpp::Duration::from_seconds(
               static_cast<double>(frame_timeout_ms_) / 1000.0);
  }

  void PublishMappedCommands(bool force_failsafe) {
    const bool estop_active = IsEstopActive();
    const bool failsafe = force_failsafe || IsFrameTimedOut() || estop_active;

    geometry_msgs::msg::Twist cmd_vel;
    wheel_leg_msgs::msg::BodyCommand body_cmd;
    std_msgs::msg::String control_mode;

    body_cmd.header.stamp = now();
    if (!failsafe) {
      cmd_vel.linear.x = GetMappedAxis(linear_x_mapping_);
      cmd_vel.angular.z = GetMappedAxis(angular_z_mapping_);
      body_cmd.body_height_offset =
          GetMappedAxis(body_height_mapping_) * body_height_scale_;
      body_cmd.yaw_rate_assist =
          GetMappedAxis(yaw_rate_assist_mapping_) * yaw_rate_assist_scale_;
      control_mode.data = GetModeLabel();
    } else if (estop_active) {
      control_mode.data = estop_mode_label_;
    } else {
      control_mode.data = safe_control_mode_;
    }

    cmd_vel_pub_->publish(cmd_vel);
    control_mode_pub_->publish(control_mode);
    body_cmd_pub_->publish(body_cmd);
  }

  void PublishStatusAndFailsafe() {
    const bool frame_timeout = IsFrameTimedOut();
    const bool receiver_online = have_valid_frame_ && !frame_timeout;
    const bool failsafe = !serial_online_ || frame_timeout;

    PublishMappedCommands(failsafe);

    wheel_leg_msgs::msg::RcStatus status;
    status.header.stamp = now();
    status.last_valid_frame_time = last_valid_frame_time_;
    status.serial_online = serial_online_;
    status.receiver_online = receiver_online;
    status.frame_timeout = frame_timeout;
    status.failsafe = failsafe;
    status.estop_active = IsEstopActive();
    status.valid_frame_count = counters_.valid_frame_count;
    status.checksum_error_count = counters_.checksum_error_count;
    status.frame_sync_loss_count = counters_.frame_sync_loss_count;
    status.serial_read_error_count = counters_.serial_read_error_count;
    status.serial_open_failure_count = counters_.serial_open_failure_count;
    status_pub_->publish(status);

    if (receiver_online) {
      RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "iBUS online: valid_frames=%u checksum_errors=%u sync_loss=%u",
          counters_.valid_frame_count,
          counters_.checksum_error_count,
          counters_.frame_sync_loss_count);
    } else {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          4000,
          "iBUS failsafe active: serial_online=%s valid_frames=%u timeout=%s",
          serial_online_ ? "true" : "false",
          counters_.valid_frame_count,
          frame_timeout ? "true" : "false");
    }
  }

  std::string serial_device_;
  int baud_rate_ = 0;
  int read_period_ms_ = 0;
  int status_period_ms_ = 0;
  int frame_timeout_ms_ = 0;
  int mode_channel_ = 0;
  int estop_channel_ = 0;
  double mode_low_threshold_ = 0.0;
  double mode_high_threshold_ = 0.0;
  double estop_threshold_ = 0.0;
  double body_height_scale_ = 0.0;
  double yaw_rate_assist_scale_ = 0.0;
  bool estop_active_below_ = true;
  std::string mode_low_label_;
  std::string mode_mid_label_;
  std::string mode_high_label_;
  std::string estop_mode_label_;
  std::string safe_control_mode_;

  ChannelMapping linear_x_mapping_;
  ChannelMapping angular_z_mapping_;
  ChannelMapping body_height_mapping_;
  ChannelMapping yaw_rate_assist_mapping_;

  int serial_fd_ = -1;
  bool serial_online_ = false;
  bool have_valid_frame_ = false;
  Counters counters_;
  rclcpp::Time last_valid_frame_time_ {0, 0, RCL_ROS_TIME};
  std::array<uint16_t, kIbusChannelCount> latest_channels_ {};
  std::vector<uint8_t> rx_buffer_;
  const std::array<uint8_t, 2> kIbusHeader_ {kIbusLengthByte, kIbusCommandByte};

  rclcpp::Publisher<wheel_leg_msgs::msg::RcChannelsRaw>::SharedPtr raw_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::RcStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::BodyCommand>::SharedPtr body_cmd_pub_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace wheel_leg_rc

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<wheel_leg_rc::RcIbusNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
