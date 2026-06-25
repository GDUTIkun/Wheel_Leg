#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

namespace wheel_leg_stm32_bridge {
namespace {

constexpr uint8_t kFrameHead0 = 0xA5;
constexpr uint8_t kFrameHead1 = 0x5A;
constexpr int kDefaultBaudRate = 921600;
constexpr double kDefaultRateHz = 200.0;
constexpr int kDefaultPayloadLen = 32;
constexpr double kDefaultReportPeriodSec = 1.0;
constexpr int kDefaultFrameType = 0x01;

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

std::vector<uint8_t> MakePayload(uint16_t seq, int payload_len) {
  std::vector<uint8_t> payload(static_cast<std::size_t>(payload_len), 0);
  for (int index = 0; index < payload_len; ++index) {
    payload[static_cast<std::size_t>(index)] =
        static_cast<uint8_t>((seq + index) & 0xFFu);
  }
  return payload;
}

std::vector<uint8_t> BuildFrame(uint8_t frame_type, uint16_t seq, int payload_len) {
  const std::vector<uint8_t> payload = MakePayload(seq, payload_len);
  const std::size_t body_size = 4u + payload.size();
  std::vector<uint8_t> frame(2u + body_size + 2u, 0);

  frame[0] = kFrameHead0;
  frame[1] = kFrameHead1;
  frame[2] = frame_type;
  frame[3] = static_cast<uint8_t>(payload.size());
  frame[4] = static_cast<uint8_t>(seq & 0xFFu);
  frame[5] = static_cast<uint8_t>((seq >> 8) & 0xFFu);
  std::copy(payload.begin(), payload.end(), frame.begin() + 6);

  const uint16_t crc = Crc16Ccitt(frame.data() + 2, body_size);
  frame[6 + payload.size()] = static_cast<uint8_t>(crc & 0xFFu);
  frame[7 + payload.size()] = static_cast<uint8_t>((crc >> 8) & 0xFFu);
  return frame;
}

struct StressCounters {
  std::atomic<uint64_t> frames_attempted{0};
  std::atomic<uint64_t> frames_sent{0};
  std::atomic<uint64_t> bytes_sent{0};
  std::atomic<uint64_t> write_errors{0};
  std::atomic<uint64_t> partial_writes{0};
  std::atomic<uint64_t> deadline_misses{0};
  std::atomic<uint32_t> last_seq{0};
};

}  // namespace

class Stm32UartStressNode : public rclcpp::Node {
 public:
  Stm32UartStressNode() : rclcpp::Node("stm32_uart_stress_node") {
    serial_device_ =
        declare_parameter<std::string>("serial_device", "/dev/ttyAMA4");
    baud_rate_ = declare_parameter<int>("baud_rate", kDefaultBaudRate);
    rate_hz_ = declare_parameter<double>("rate_hz", kDefaultRateHz);
    payload_len_ = declare_parameter<int>("payload_len", kDefaultPayloadLen);
    frame_type_ = declare_parameter<int>("frame_type", kDefaultFrameType);
    report_period_sec_ =
        declare_parameter<double>("report_period_sec", kDefaultReportPeriodSec);

    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/stm32_bridge/uart_stress/status_text", 10);
    counters_pub_ = create_publisher<std_msgs::msg::UInt32MultiArray>(
        "/stm32_bridge/uart_stress/counters", 10);

    if (!ValidateParameters()) {
      throw std::runtime_error("invalid stm32 uart stress parameters");
    }
    if (!OpenSerialPort()) {
      throw std::runtime_error("failed to open stm32 uart stress serial port");
    }

    running_.store(true);
    writer_thread_ = std::thread([this]() { WriterLoop(); });

    report_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(report_period_sec_)),
        [this]() { PublishStatus(); });

    RCLCPP_INFO(
        get_logger(),
        "Started STM32 UART stress node on %s at %d baud, rate %.1f Hz, payload_len=%d",
        serial_device_.c_str(),
        baud_rate_,
        rate_hz_,
        payload_len_);
  }

  ~Stm32UartStressNode() override {
    running_.store(false);
    if (writer_thread_.joinable()) {
      writer_thread_.join();
    }
    CloseSerialPort();
  }

 private:
  bool ValidateParameters() {
    if (rate_hz_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "rate_hz must be positive");
      return false;
    }
    if (payload_len_ < 0 || payload_len_ > 96) {
      RCLCPP_ERROR(get_logger(), "payload_len must be in 0..96");
      return false;
    }
    if (frame_type_ < 0 || frame_type_ > 0xFF) {
      RCLCPP_ERROR(get_logger(), "frame_type must be in 0..255");
      return false;
    }
    if (report_period_sec_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "report_period_sec must be positive");
      return false;
    }
    return true;
  }

  bool OpenSerialPort() {
    const speed_t baud = ToTermiosBaudRate(baud_rate_);
    if (baud == 0) {
      RCLCPP_ERROR(
          get_logger(),
          "Unsupported baud_rate=%d on this platform",
          baud_rate_);
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
    tty.c_cc[VTIME] = 0;

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
    return true;
  }

  void CloseSerialPort() {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  void WriterLoop() {
    uint16_t seq = 0;
    auto next_deadline = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);

    while (running_.load()) {
      next_deadline += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      counters_.frames_attempted.fetch_add(1, std::memory_order_relaxed);

      const std::vector<uint8_t> frame =
          BuildFrame(static_cast<uint8_t>(frame_type_), seq, payload_len_);
      const ssize_t written = write(serial_fd_, frame.data(), frame.size());
      if (written < 0) {
        counters_.write_errors.fetch_add(1, std::memory_order_relaxed);
      } else {
        counters_.bytes_sent.fetch_add(
            static_cast<uint64_t>(written), std::memory_order_relaxed);
        if (static_cast<std::size_t>(written) == frame.size()) {
          counters_.frames_sent.fetch_add(1, std::memory_order_relaxed);
          counters_.last_seq.store(seq, std::memory_order_relaxed);
        } else {
          counters_.partial_writes.fetch_add(1, std::memory_order_relaxed);
        }
      }
      seq = static_cast<uint16_t>(seq + 1u);

      std::this_thread::sleep_until(next_deadline);
      const auto now = std::chrono::steady_clock::now();
      if (now > next_deadline + std::chrono::microseconds(500)) {
        counters_.deadline_misses.fetch_add(1, std::memory_order_relaxed);
        next_deadline = now;
      }
    }
  }

  void PublishStatus() {
    const uint64_t frames_attempted =
        counters_.frames_attempted.load(std::memory_order_relaxed);
    const uint64_t frames_sent =
        counters_.frames_sent.load(std::memory_order_relaxed);
    const uint64_t bytes_sent =
        counters_.bytes_sent.load(std::memory_order_relaxed);
    const uint64_t write_errors =
        counters_.write_errors.load(std::memory_order_relaxed);
    const uint64_t partial_writes =
        counters_.partial_writes.load(std::memory_order_relaxed);
    const uint64_t deadline_misses =
        counters_.deadline_misses.load(std::memory_order_relaxed);
    const uint32_t last_seq = counters_.last_seq.load(std::memory_order_relaxed);

    std_msgs::msg::UInt32MultiArray counters_msg;
    counters_msg.data = {
        static_cast<uint32_t>(frames_attempted),
        static_cast<uint32_t>(frames_sent),
        static_cast<uint32_t>(bytes_sent),
        static_cast<uint32_t>(write_errors),
        static_cast<uint32_t>(partial_writes),
        static_cast<uint32_t>(deadline_misses),
        last_seq};
    counters_pub_->publish(counters_msg);

    std_msgs::msg::String status_msg;
    status_msg.data =
        "device=" + serial_device_ +
        " baud=" + std::to_string(baud_rate_) +
        " rate_hz=" + std::to_string(rate_hz_) +
        " payload_len=" + std::to_string(payload_len_) +
        " attempted=" + std::to_string(frames_attempted) +
        " sent=" + std::to_string(frames_sent) +
        " bytes=" + std::to_string(bytes_sent) +
        " write_errors=" + std::to_string(write_errors) +
        " partial_writes=" + std::to_string(partial_writes) +
        " deadline_misses=" + std::to_string(deadline_misses) +
        " last_seq=" + std::to_string(last_seq);
    status_pub_->publish(status_msg);

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "%s",
        status_msg.data.c_str());
  }

  std::string serial_device_;
  int baud_rate_ = kDefaultBaudRate;
  double rate_hz_ = kDefaultRateHz;
  int payload_len_ = kDefaultPayloadLen;
  int frame_type_ = kDefaultFrameType;
  double report_period_sec_ = kDefaultReportPeriodSec;
  int serial_fd_ = -1;
  std::atomic<bool> running_{false};
  StressCounters counters_;
  std::thread writer_thread_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr counters_pub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

}  // namespace wheel_leg_stm32_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<wheel_leg_stm32_bridge::Stm32UartStressNode>());
  rclcpp::shutdown();
  return 0;
}
