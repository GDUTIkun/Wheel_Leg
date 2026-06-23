#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

namespace wheel_leg_rc {
namespace {

constexpr const char* kDefaultSerialDevice = "/dev/ttyAMA3";
constexpr int kDefaultBaudRate = 115200;
constexpr int kDefaultReportPeriodMs = 1000;
constexpr int kDefaultReadTimeoutMs = 100;
constexpr int kDefaultHexPreviewBytes = 16;
constexpr int kDefaultMinNonZeroBytesPerReport = 1;

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

std::string HexDumpPreview(
    const std::vector<uint8_t>& buffer,
    std::size_t max_bytes) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  const std::size_t preview_size = std::min(buffer.size(), max_bytes);
  for (std::size_t index = 0; index < preview_size; ++index) {
    if (index > 0) {
      stream << ' ';
    }
    stream << std::setw(2) << static_cast<int>(buffer[index]);
  }
  return stream.str();
}

std::string AsciiPreview(
    const std::vector<uint8_t>& buffer,
    std::size_t max_bytes) {
  std::string preview;
  const std::size_t preview_size = std::min(buffer.size(), max_bytes);
  preview.reserve(preview_size);
  for (std::size_t index = 0; index < preview_size; ++index) {
    const unsigned char value = buffer[index];
    preview.push_back(std::isprint(value) ? static_cast<char>(value) : '.');
  }
  return preview;
}

struct ProbeCounters {
  uint64_t total_bytes = 0;
  uint64_t nonzero_bytes = 0;
  uint64_t read_calls = 0;
  uint64_t nonempty_reads = 0;
  uint64_t open_failures = 0;
  uint64_t read_errors = 0;
};

}  // namespace

class RcSerialProbeNode : public rclcpp::Node {
 public:
  RcSerialProbeNode()
      : rclcpp::Node("rc_serial_probe_node") {
    serial_device_ =
        declare_parameter<std::string>("serial_device", kDefaultSerialDevice);
    baud_rate_ = declare_parameter<int>("baud_rate", kDefaultBaudRate);
    report_period_ms_ =
        declare_parameter<int>("report_period_ms", kDefaultReportPeriodMs);
    read_timeout_ms_ =
        declare_parameter<int>("read_timeout_ms", kDefaultReadTimeoutMs);
    hex_preview_bytes_ =
        declare_parameter<int>("hex_preview_bytes", kDefaultHexPreviewBytes);
    min_nonzero_bytes_per_report_ = declare_parameter<int>(
        "min_nonzero_bytes_per_report",
        kDefaultMinNonZeroBytesPerReport);

    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/rc/serial_probe/status_text", 10);
    counters_pub_ =
        create_publisher<std_msgs::msg::UInt32MultiArray>(
            "/rc/serial_probe/counters", 10);

    if (!OpenSerialPort()) {
      PublishStatus("serial_open_failed");
    }

    read_timer_ = create_wall_timer(
        std::chrono::milliseconds(read_timeout_ms_),
        [this]() { PollSerialPort(); });
    report_timer_ = create_wall_timer(
        std::chrono::milliseconds(report_period_ms_),
        [this]() { ReportProbeStatus(); });
  }

  ~RcSerialProbeNode() override {
    CloseSerialPort();
  }

 private:
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
      ++counters_.open_failures;
      RCLCPP_ERROR(
          get_logger(),
          "Failed to open %s: %s",
          serial_device_.c_str(),
          std::strerror(errno));
      return false;
    }

    termios tty {};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      ++counters_.open_failures;
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
      ++counters_.open_failures;
      RCLCPP_ERROR(
          get_logger(),
          "tcsetattr(%s) failed: %s",
          serial_device_.c_str(),
          std::strerror(errno));
      CloseSerialPort();
      return false;
    }

    tcflush(serial_fd_, TCIFLUSH);
    RCLCPP_INFO(
        get_logger(),
        "Opened %s with %d 8N1 for RC serial probing",
        serial_device_.c_str(),
        baud_rate_);
    return true;
  }

  void CloseSerialPort() {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  void PollSerialPort() {
    if (serial_fd_ < 0) {
      if (OpenSerialPort()) {
        PublishStatus("serial_reopened");
      }
      return;
    }

    std::array<uint8_t, 256> buffer {};
    const ssize_t bytes_read = read(serial_fd_, buffer.data(), buffer.size());
    ++counters_.read_calls;

    if (bytes_read < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }

      ++counters_.read_errors;
      RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Read error on %s: %s",
          serial_device_.c_str(),
          std::strerror(errno));
      CloseSerialPort();
      PublishStatus("serial_read_error");
      return;
    }

    if (bytes_read == 0) {
      return;
    }

    ++counters_.nonempty_reads;
    counters_.total_bytes += static_cast<uint64_t>(bytes_read);

    last_chunk_.assign(buffer.begin(), buffer.begin() + bytes_read);
    for (const uint8_t byte : last_chunk_) {
      if (byte != 0) {
        ++counters_.nonzero_bytes;
      }
    }
    last_read_time_ = now();
  }

  void ReportProbeStatus() {
    const rclcpp::Time current_time = now();
    const bool port_open = serial_fd_ >= 0;
    const bool has_recent_data =
        last_read_time_.nanoseconds() > 0 &&
        (current_time - last_read_time_) <
            rclcpp::Duration::from_seconds(
                static_cast<double>(report_period_ms_) / 1000.0 * 2.0);
    const bool saw_nonzero_bytes =
        counters_.nonzero_bytes >=
        static_cast<uint64_t>(std::max(0, min_nonzero_bytes_per_report_));

    std::ostringstream status_stream;
    status_stream
        << "device=" << serial_device_
        << " baud=" << baud_rate_
        << " open=" << (port_open ? "true" : "false")
        << " recent_data=" << (has_recent_data ? "true" : "false")
        << " total_bytes=" << counters_.total_bytes
        << " nonzero_bytes=" << counters_.nonzero_bytes
        << " nonempty_reads=" << counters_.nonempty_reads
        << " read_errors=" << counters_.read_errors
        << " open_failures=" << counters_.open_failures;

    if (!last_chunk_.empty()) {
      status_stream
          << " hex=[" << HexDumpPreview(last_chunk_, hex_preview_bytes_) << "]"
          << " ascii=[" << AsciiPreview(last_chunk_, hex_preview_bytes_) << "]";
    }

    if (port_open && has_recent_data && saw_nonzero_bytes) {
      RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "RC serial activity detected: %s",
          status_stream.str().c_str());
    } else {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          4000,
          "RC serial probe is waiting for data: %s",
          status_stream.str().c_str());
    }

    PublishStatus(status_stream.str());
    PublishCounters(port_open, has_recent_data, saw_nonzero_bytes);
  }

  void PublishStatus(const std::string& status_text) {
    std_msgs::msg::String message;
    message.data = status_text;
    status_pub_->publish(message);
  }

  void PublishCounters(
      bool port_open,
      bool has_recent_data,
      bool saw_nonzero_bytes) {
    std_msgs::msg::UInt32MultiArray message;
    message.layout.dim.resize(1);
    message.layout.dim[0].label =
        "port_open,recent_data,saw_nonzero,total_bytes,nonzero_bytes,nonempty_reads,read_errors,open_failures,last_chunk_bytes";
    message.layout.dim[0].size = 9;
    message.layout.dim[0].stride = 9;
    message.data = {
        static_cast<uint32_t>(port_open ? 1U : 0U),
        static_cast<uint32_t>(has_recent_data ? 1U : 0U),
        static_cast<uint32_t>(saw_nonzero_bytes ? 1U : 0U),
        static_cast<uint32_t>(
            std::min<uint64_t>(counters_.total_bytes, UINT32_MAX)),
        static_cast<uint32_t>(
            std::min<uint64_t>(counters_.nonzero_bytes, UINT32_MAX)),
        static_cast<uint32_t>(
            std::min<uint64_t>(counters_.nonempty_reads, UINT32_MAX)),
        static_cast<uint32_t>(
            std::min<uint64_t>(counters_.read_errors, UINT32_MAX)),
        static_cast<uint32_t>(
            std::min<uint64_t>(counters_.open_failures, UINT32_MAX)),
        static_cast<uint32_t>(
            std::min<std::size_t>(last_chunk_.size(), UINT32_MAX)),
    };
    counters_pub_->publish(message);
  }

  std::string serial_device_;
  int baud_rate_ = 0;
  int report_period_ms_ = 0;
  int read_timeout_ms_ = 0;
  int hex_preview_bytes_ = 0;
  int min_nonzero_bytes_per_report_ = 0;

  int serial_fd_ = -1;
  ProbeCounters counters_;
  rclcpp::Time last_read_time_ {0, 0, RCL_ROS_TIME};
  std::vector<uint8_t> last_chunk_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr counters_pub_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

}  // namespace wheel_leg_rc

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<wheel_leg_rc::RcSerialProbeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
