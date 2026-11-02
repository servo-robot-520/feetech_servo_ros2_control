#pragma once

#include <chrono>
#include <cstdint>
#include <span>
#include <string>

#include "feetech_servo_ros2_control/result.hpp"

namespace feetech_servo_ros2_control {

/// Linux serial transport for Feetech TTL bus servos (8N1, raw mode).
///
/// The port is opened non-blocking; every read has an explicit deadline so a
/// missing servo response can never stall the control loop.
class ScsSerialPort {
 public:
  ScsSerialPort() = default;
  ~ScsSerialPort();

  ScsSerialPort(const ScsSerialPort&) = delete;
  ScsSerialPort& operator=(const ScsSerialPort&) = delete;

  Result open(const std::string& device, int baud_rate);
  void close();
  bool is_open() const { return fd_ >= 0; }

  Result write(std::span<const uint8_t> data);
  Result read_exact(std::span<uint8_t> data, std::chrono::milliseconds timeout);
  void flush_input();

 private:
  int fd_{-1};
};

}  // namespace feetech_servo_ros2_control
