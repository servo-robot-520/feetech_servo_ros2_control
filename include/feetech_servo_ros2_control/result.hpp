#pragma once

#include <string>
#include <utility>

namespace feetech_servo_ros2_control {

/// Minimal result type: either success or an error message.
class Result {
 public:
  Result() = default;

  static Result success() { return Result{}; }

  static Result failure(std::string message) {
    Result result;
    result.ok_ = false;
    result.message_ = std::move(message);
    return result;
  }

  bool has_value() const { return ok_; }
  explicit operator bool() const { return ok_; }
  const std::string& error() const { return message_; }

 private:
  bool ok_{true};
  std::string message_;
};

/// Minimal expected type: either a value or an error message.
template <typename T>
class Expected {
 public:
  explicit Expected(T value) : value_(std::move(value)) {}

  static Expected failure(std::string message) {
    Expected result{T{}};
    result.has_value_ = false;
    result.message_ = std::move(message);
    return result;
  }

  bool has_value() const { return has_value_; }
  explicit operator bool() const { return has_value_; }
  const T& value() const { return value_; }
  const std::string& error() const { return message_; }

 private:
  T value_{};
  std::string message_;
  bool has_value_{true};
};

}  // namespace feetech_servo_ros2_control
