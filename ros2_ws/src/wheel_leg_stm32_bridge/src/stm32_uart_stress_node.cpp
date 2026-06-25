#include <algorithm>
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
#include <mutex>
#include <stdexcept>
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
constexpr uint8_t kFrameTypeHostToStm = 0x01;
constexpr uint8_t kFrameTypeStmStatus = 0x81;
constexpr uint8_t kMaxPayloadLen = 96;
constexpr int kDefaultBaudRate = 921600;
constexpr double kDefaultRateHz = 200.0;
constexpr int kDefaultPayloadLen = 32;
constexpr double kDefaultReportPeriodSec = 1.0;

struct Stm32StatusFrame {
  uint32_t stm_tick_ms = 0;
  uint32_t rx_bytes = 0;
  uint32_t frames_ok = 0;
  uint32_t crc_errors = 0;
  uint32_t length_errors = 0;
  uint32_t sync_losses = 0;
  uint32_t rx_seq_gaps = 0;
  uint32_t uart_errors = 0;
  uint16_t last_rx_seq = 0;
  uint8_t last_rx_type = 0;
  uint8_t last_rx_len = 0;
  uint32_t min_frame_gap_ms = 0;
  uint32_t max_frame_gap_ms = 0;
  uint32_t last_rx_age_ms = 0;
};

struct TxCounters {
  std::atomic<uint64_t> frames_attempted{0};
  std::atomic<uint64_t> frames_sent{0};
  std::atomic<uint64_t> bytes_sent{0};
  std::atomic<uint64_t> write_errors{0};
  std::atomic<uint64_t> partial_writes{0};
  std::atomic<uint64_t> deadline_misses{0};
  std::atomic<uint32_t> last_seq{0};
};

struct RxCounters {
  std::atomic<uint64_t> bytes_received{0};
  std::atomic<uint64_t> frames_ok{0};
  std::atomic<uint64_t> crc_errors{0};
  std::atomic<uint64_t> length_errors{0};
  std::atomic<uint64_t> sync_losses{0};
  std::atomic<uint64_t> short_reads{0};
  std::atomic<uint64_t> read_errors{0};
  std::atomic<uint64_t> unknown_types{0};
  std::atomic<uint32_t> last_seq{0};
  std::atomic<uint32_t> last_payload_len{0};
};

enum class RxState {
  kHead0,
  kHead1,
  kType,
  kLen,
  kSeqLo,
  kSeqHi,
  kPayload,
  kCrcLo,
  kCrcHi,
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

bool DecodeStm32StatusPayload(const std::vector<uint8_t>& payload,
                              Stm32StatusFrame* frame) {
  if (frame == nullptr || payload.size() != 48u) {
    return false;
  }

  frame->stm_tick_ms = ReadU32Le(payload.data() + 0);
  frame->rx_bytes = ReadU32Le(payload.data() + 4);
  frame->frames_ok = ReadU32Le(payload.data() + 8);
  frame->crc_errors = ReadU32Le(payload.data() + 12);
  frame->length_errors = ReadU32Le(payload.data() + 16);
  frame->sync_losses = ReadU32Le(payload.data() + 20);
  frame->rx_seq_gaps = ReadU32Le(payload.data() + 24);
  frame->uart_errors = ReadU32Le(payload.data() + 28);
  frame->last_rx_seq = ReadU16Le(payload.data() + 32);
  frame->last_rx_type = payload[34];
  frame->last_rx_len = payload[35];
  frame->min_frame_gap_ms = ReadU32Le(payload.data() + 36);
  frame->max_frame_gap_ms = ReadU32Le(payload.data() + 40);
  frame->last_rx_age_ms = ReadU32Le(payload.data() + 44);
  return true;
}

}  // namespace

class Stm32UartStressNode : public rclcpp::Node {
 public:
  Stm32UartStressNode() : rclcpp::Node("stm32_uart_stress_node") {
    serial_device_ =
        declare_parameter<std::string>("serial_device", "/dev/ttyAMA4");
    baud_rate_ = declare_parameter<int>("baud_rate", kDefaultBaudRate);
    rate_hz_ = declare_parameter<double>("rate_hz", kDefaultRateHz);
    payload_len_ = declare_parameter<int>("payload_len", kDefaultPayloadLen);
    frame_type_ = declare_parameter<int>("frame_type", kFrameTypeHostToStm);
    report_period_sec_ =
        declare_parameter<double>("report_period_sec", kDefaultReportPeriodSec);
    enable_tx_ = declare_parameter<bool>("enable_tx", true);
    enable_rx_ = declare_parameter<bool>("enable_rx", true);

    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/stm32_bridge/uart_stress/status_text", 10);
    tx_counters_pub_ = create_publisher<std_msgs::msg::UInt32MultiArray>(
        "/stm32_bridge/uart_stress/tx_counters", 10);
    rx_counters_pub_ = create_publisher<std_msgs::msg::UInt32MultiArray>(
        "/stm32_bridge/uart_stress/rx_counters", 10);
    stm32_status_pub_ = create_publisher<std_msgs::msg::UInt32MultiArray>(
        "/stm32_bridge/uart_stress/stm32_status", 10);

    if (!ValidateParameters()) {
      throw std::runtime_error("invalid stm32 uart stress parameters");
    }
    if (!OpenSerialPort()) {
      throw std::runtime_error("failed to open stm32 uart stress serial port");
    }

    running_.store(true);
    if (enable_tx_) {
      writer_thread_ = std::thread([this]() { WriterLoop(); });
    }
    if (enable_rx_) {
      reader_thread_ = std::thread([this]() { ReaderLoop(); });
    }

    report_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(report_period_sec_)),
        [this]() { PublishStatus(); });

    RCLCPP_INFO(
        get_logger(),
        "Started STM32 UART stress node on %s at %d baud, tx=%s rx=%s rate=%.1f Hz payload_len=%d",
        serial_device_.c_str(),
        baud_rate_,
        enable_tx_ ? "on" : "off",
        enable_rx_ ? "on" : "off",
        rate_hz_,
        payload_len_);
  }

  ~Stm32UartStressNode() override {
    running_.store(false);
    if (writer_thread_.joinable()) {
      writer_thread_.join();
    }
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
    CloseSerialPort();
  }

 private:
  bool ValidateParameters() {
    if (rate_hz_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "rate_hz must be positive");
      return false;
    }
    if (payload_len_ < 0 || payload_len_ > kMaxPayloadLen) {
      RCLCPP_ERROR(
          get_logger(), "payload_len must be in 0..%d", kMaxPayloadLen);
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
          get_logger(), "Unsupported baud_rate=%d on this platform", baud_rate_);
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
      next_deadline +=
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      tx_counters_.frames_attempted.fetch_add(1, std::memory_order_relaxed);

      const std::vector<uint8_t> frame =
          BuildFrame(static_cast<uint8_t>(frame_type_), seq, payload_len_);
      const ssize_t written = write(serial_fd_, frame.data(), frame.size());
      if (written < 0) {
        tx_counters_.write_errors.fetch_add(1, std::memory_order_relaxed);
      } else {
        tx_counters_.bytes_sent.fetch_add(
            static_cast<uint64_t>(written), std::memory_order_relaxed);
        if (static_cast<std::size_t>(written) == frame.size()) {
          tx_counters_.frames_sent.fetch_add(1, std::memory_order_relaxed);
          tx_counters_.last_seq.store(seq, std::memory_order_relaxed);
        } else {
          tx_counters_.partial_writes.fetch_add(1, std::memory_order_relaxed);
        }
      }
      seq = static_cast<uint16_t>(seq + 1u);

      std::this_thread::sleep_until(next_deadline);
      const auto now = std::chrono::steady_clock::now();
      if (now > next_deadline + std::chrono::microseconds(500)) {
        tx_counters_.deadline_misses.fetch_add(1, std::memory_order_relaxed);
        next_deadline = now;
      }
    }
  }

  void ResetParser() {
    rx_state_ = RxState::kHead0;
    rx_frame_type_ = 0;
    rx_payload_len_ = 0;
    rx_payload_index_ = 0;
    rx_seq_ = 0;
    rx_crc_ = 0;
  }

  void HandleCompleteFrame() {
    rx_counters_.frames_ok.fetch_add(1, std::memory_order_relaxed);
    rx_counters_.last_seq.store(rx_seq_, std::memory_order_relaxed);
    rx_counters_.last_payload_len.store(rx_payload_len_, std::memory_order_relaxed);

    if (rx_frame_type_ == kFrameTypeStmStatus) {
      Stm32StatusFrame status;
      if (!DecodeStm32StatusPayload(rx_payload_, &status)) {
        rx_counters_.length_errors.fetch_add(1, std::memory_order_relaxed);
        return;
      }

      {
        std::lock_guard<std::mutex> lock(last_stm32_status_mutex_);
        last_stm32_status_ = status;
      }
      have_stm32_status_.store(true, std::memory_order_relaxed);
    } else {
      rx_counters_.unknown_types.fetch_add(1, std::memory_order_relaxed);
    }
  }

  void ConsumeByte(uint8_t byte) {
    rx_counters_.bytes_received.fetch_add(1, std::memory_order_relaxed);

    switch (rx_state_) {
      case RxState::kHead0:
        if (byte == kFrameHead0) {
          rx_state_ = RxState::kHead1;
        } else {
          rx_counters_.sync_losses.fetch_add(1, std::memory_order_relaxed);
        }
        break;

      case RxState::kHead1:
        if (byte == kFrameHead1) {
          rx_state_ = RxState::kType;
        } else {
          rx_counters_.sync_losses.fetch_add(1, std::memory_order_relaxed);
          rx_state_ = (byte == kFrameHead0) ? RxState::kHead1 : RxState::kHead0;
        }
        break;

      case RxState::kType:
        rx_frame_type_ = byte;
        rx_state_ = RxState::kLen;
        break;

      case RxState::kLen:
        rx_payload_len_ = byte;
        if (rx_payload_len_ > kMaxPayloadLen) {
          rx_counters_.length_errors.fetch_add(1, std::memory_order_relaxed);
          ResetParser();
        } else {
          rx_payload_.assign(rx_payload_len_, 0);
          rx_payload_index_ = 0;
          rx_state_ = RxState::kSeqLo;
        }
        break;

      case RxState::kSeqLo:
        rx_seq_ = byte;
        rx_state_ = RxState::kSeqHi;
        break;

      case RxState::kSeqHi:
        rx_seq_ |= static_cast<uint16_t>(byte) << 8;
        rx_state_ = (rx_payload_len_ == 0u) ? RxState::kCrcLo : RxState::kPayload;
        break;

      case RxState::kPayload:
        rx_payload_[rx_payload_index_++] = byte;
        if (rx_payload_index_ >= rx_payload_len_) {
          rx_state_ = RxState::kCrcLo;
        }
        break;

      case RxState::kCrcLo:
        rx_crc_ = byte;
        rx_state_ = RxState::kCrcHi;
        break;

      case RxState::kCrcHi: {
        rx_crc_ |= static_cast<uint16_t>(byte) << 8;
        std::vector<uint8_t> body(4u + rx_payload_.size(), 0);
        body[0] = rx_frame_type_;
        body[1] = rx_payload_len_;
        body[2] = static_cast<uint8_t>(rx_seq_ & 0xFFu);
        body[3] = static_cast<uint8_t>((rx_seq_ >> 8) & 0xFFu);
        std::copy(rx_payload_.begin(), rx_payload_.end(), body.begin() + 4);
        const uint16_t expected_crc = Crc16Ccitt(body.data(), body.size());
        if (expected_crc == rx_crc_) {
          HandleCompleteFrame();
        } else {
          rx_counters_.crc_errors.fetch_add(1, std::memory_order_relaxed);
        }
        ResetParser();
        break;
      }
    }
  }

  void ReaderLoop() {
    std::array<uint8_t, 256> buffer {};
    ResetParser();

    while (running_.load()) {
      const ssize_t read_count = read(serial_fd_, buffer.data(), buffer.size());
      if (read_count < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
          continue;
        }
        rx_counters_.read_errors.fetch_add(1, std::memory_order_relaxed);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      if (read_count == 0) {
        rx_counters_.short_reads.fetch_add(1, std::memory_order_relaxed);
        continue;
      }

      for (ssize_t index = 0; index < read_count; ++index) {
        ConsumeByte(buffer[static_cast<std::size_t>(index)]);
      }
    }
  }

  void PublishStatus() {
    const uint64_t frames_attempted =
        tx_counters_.frames_attempted.load(std::memory_order_relaxed);
    const uint64_t frames_sent =
        tx_counters_.frames_sent.load(std::memory_order_relaxed);
    const uint64_t bytes_sent =
        tx_counters_.bytes_sent.load(std::memory_order_relaxed);
    const uint64_t write_errors =
        tx_counters_.write_errors.load(std::memory_order_relaxed);
    const uint64_t partial_writes =
        tx_counters_.partial_writes.load(std::memory_order_relaxed);
    const uint64_t deadline_misses =
        tx_counters_.deadline_misses.load(std::memory_order_relaxed);
    const uint32_t last_tx_seq =
        tx_counters_.last_seq.load(std::memory_order_relaxed);

    const uint64_t bytes_received =
        rx_counters_.bytes_received.load(std::memory_order_relaxed);
    const uint64_t rx_frames_ok =
        rx_counters_.frames_ok.load(std::memory_order_relaxed);
    const uint64_t rx_crc_errors =
        rx_counters_.crc_errors.load(std::memory_order_relaxed);
    const uint64_t rx_length_errors =
        rx_counters_.length_errors.load(std::memory_order_relaxed);
    const uint64_t rx_sync_losses =
        rx_counters_.sync_losses.load(std::memory_order_relaxed);
    const uint64_t read_errors =
        rx_counters_.read_errors.load(std::memory_order_relaxed);
    const uint64_t unknown_types =
        rx_counters_.unknown_types.load(std::memory_order_relaxed);
    const uint32_t last_rx_seq =
        rx_counters_.last_seq.load(std::memory_order_relaxed);

    std_msgs::msg::UInt32MultiArray tx_msg;
    tx_msg.data = {
        static_cast<uint32_t>(frames_attempted),
        static_cast<uint32_t>(frames_sent),
        static_cast<uint32_t>(bytes_sent),
        static_cast<uint32_t>(write_errors),
        static_cast<uint32_t>(partial_writes),
        static_cast<uint32_t>(deadline_misses),
        last_tx_seq};
    tx_counters_pub_->publish(tx_msg);

    std_msgs::msg::UInt32MultiArray rx_msg;
    rx_msg.data = {
        static_cast<uint32_t>(bytes_received),
        static_cast<uint32_t>(rx_frames_ok),
        static_cast<uint32_t>(rx_crc_errors),
        static_cast<uint32_t>(rx_length_errors),
        static_cast<uint32_t>(rx_sync_losses),
        static_cast<uint32_t>(read_errors),
        static_cast<uint32_t>(unknown_types),
        last_rx_seq};
    rx_counters_pub_->publish(rx_msg);

    std::string stm32_summary = "stm32_status=none";
    if (have_stm32_status_.load(std::memory_order_relaxed)) {
      Stm32StatusFrame stm32_status;
      {
        std::lock_guard<std::mutex> lock(last_stm32_status_mutex_);
        stm32_status = last_stm32_status_;
      }

      std_msgs::msg::UInt32MultiArray stm_msg;
      stm_msg.data = {
          stm32_status.stm_tick_ms,
          stm32_status.rx_bytes,
          stm32_status.frames_ok,
          stm32_status.crc_errors,
          stm32_status.length_errors,
          stm32_status.sync_losses,
          stm32_status.rx_seq_gaps,
          stm32_status.uart_errors,
          static_cast<uint32_t>(stm32_status.last_rx_seq),
          static_cast<uint32_t>(stm32_status.last_rx_type),
          static_cast<uint32_t>(stm32_status.last_rx_len),
          stm32_status.min_frame_gap_ms,
          stm32_status.max_frame_gap_ms,
          stm32_status.last_rx_age_ms};
      stm32_status_pub_->publish(stm_msg);

      stm32_summary =
          "stm_tick_ms=" + std::to_string(stm32_status.stm_tick_ms) +
          " stm_rx_ok=" + std::to_string(stm32_status.frames_ok) +
          " stm_crc=" + std::to_string(stm32_status.crc_errors) +
          " stm_len=" + std::to_string(stm32_status.length_errors) +
          " stm_sync=" + std::to_string(stm32_status.sync_losses) +
          " stm_gap=" + std::to_string(stm32_status.rx_seq_gaps) +
          " stm_uart=" + std::to_string(stm32_status.uart_errors) +
          " stm_last_age_ms=" + std::to_string(stm32_status.last_rx_age_ms);
    }

    std_msgs::msg::String status_msg;
    status_msg.data =
        "device=" + serial_device_ +
        " baud=" + std::to_string(baud_rate_) +
        " tx=" + (enable_tx_ ? std::string("on") : std::string("off")) +
        " rx=" + (enable_rx_ ? std::string("on") : std::string("off")) +
        " tx_sent=" + std::to_string(frames_sent) +
        " tx_err=" + std::to_string(write_errors) +
        " rx_ok=" + std::to_string(rx_frames_ok) +
        " rx_crc=" + std::to_string(rx_crc_errors) +
        " rx_sync=" + std::to_string(rx_sync_losses) +
        " rx_unknown=" + std::to_string(unknown_types) +
        " host_last_rx_seq=" + std::to_string(last_rx_seq) +
        " " + stm32_summary;
    status_pub_->publish(status_msg);

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s", status_msg.data.c_str());
  }

  std::string serial_device_;
  int baud_rate_ = kDefaultBaudRate;
  double rate_hz_ = kDefaultRateHz;
  int payload_len_ = kDefaultPayloadLen;
  int frame_type_ = kFrameTypeHostToStm;
  double report_period_sec_ = kDefaultReportPeriodSec;
  bool enable_tx_ = true;
  bool enable_rx_ = true;
  int serial_fd_ = -1;
  std::atomic<bool> running_{false};
  TxCounters tx_counters_;
  RxCounters rx_counters_;
  std::thread writer_thread_;
  std::thread reader_thread_;
  RxState rx_state_ = RxState::kHead0;
  uint8_t rx_frame_type_ = 0;
  uint8_t rx_payload_len_ = 0;
  uint8_t rx_payload_index_ = 0;
  uint16_t rx_seq_ = 0;
  uint16_t rx_crc_ = 0;
  std::vector<uint8_t> rx_payload_;
  std::atomic<bool> have_stm32_status_{false};
  std::mutex last_stm32_status_mutex_;
  Stm32StatusFrame last_stm32_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr tx_counters_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr rx_counters_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr stm32_status_pub_;
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
