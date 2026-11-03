#include "feetech_servo_ros2_control/scs_serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace feetech_servo_ros2_control {
    namespace {

        /// @brief 拼接带 errno 描述的错误信息。
        std::string errno_message(const std::string& what) {
            return what + ": " + std::strerror(errno);
        }

        /// @brief 根据截止时间计算 poll 的等待毫秒数，至少保留 1 ms。
        int poll_timeout_ms(const std::chrono::steady_clock::time_point deadline) {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now());
            return static_cast<int>(std::max<long long>(1, remaining.count()));
        }

    } // namespace

    ScsSerialPort::~ScsSerialPort() {
        close();
    }

    Result ScsSerialPort::open(const std::string& device, const int baud_rate) {
        if (baud_rate <= 0) {
            return Result::failure("Invalid baud rate " + std::to_string(baud_rate));
        }

        close();

        const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            return Result::failure(errno_message("Failed to open '" + device + "'"));
        }

        // 配置为 8N1、无流控、原始模式；使用 BOTHER 以支持任意波特率。
        struct termios2 tty {};
        if (::ioctl(fd, TCGETS2, &tty) != 0) {
            const auto message = errno_message("Failed to read serial settings from '" + device + "'");
            ::close(fd);
            return Result::failure(message);
        }

        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS | CBAUD);
        tty.c_cflag |= CS8 | CLOCAL | CREAD | BOTHER;
        tty.c_line = 0;
        tty.c_ispeed = static_cast<speed_t>(baud_rate);
        tty.c_ospeed = static_cast<speed_t>(baud_rate);
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (::ioctl(fd, TCSETS2, &tty) != 0) {
            const auto message =
                errno_message("Failed to configure '" + device + "' at " + std::to_string(baud_rate) + " baud");
            ::close(fd);
            return Result::failure(message);
        }

        if (::ioctl(fd, TCFLSH, TCIOFLUSH) != 0) {
            const auto message = errno_message("Failed to flush '" + device + "'");
            ::close(fd);
            return Result::failure(message);
        }

        fd_ = fd;
        return Result::success();
    }

    void ScsSerialPort::close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    Result ScsSerialPort::write(const std::span<const uint8_t> data) const {
        if (!is_open()) {
            return Result::failure("Serial port is not open");
        }

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
        size_t offset = 0;
        while (offset < data.size()) {
            const ssize_t written = ::write(fd_, data.data() + offset, data.size() - offset);
            if (written > 0) {
                offset += static_cast<size_t>(written);
                continue;
            }
            if (written < 0 && errno == EINTR) {
                continue;
            }
            if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                pollfd descriptor{.fd = fd_, .events = POLLOUT, .revents = 0};
                if (const int ready = ::poll(&descriptor, 1, poll_timeout_ms(deadline)); ready < 0 && errno != EINTR) {
                    return Result::failure(errno_message("Failed to wait for serial port"));
                }
                if (std::chrono::steady_clock::now() >= deadline) {
                    return Result::failure("Timed out writing to serial port");
                }
                continue;
            }
            return Result::failure(errno_message("Failed to write to serial port"));
        }
        return Result::success();
    }

    Result ScsSerialPort::read_exact(const std::span<uint8_t> data,
                                     const std::chrono::milliseconds timeout) const {
        if (!is_open()) {
            return Result::failure("Serial port is not open");
        }

        const auto deadline = std::chrono::steady_clock::now() + timeout;
        size_t offset = 0;
        while (offset < data.size()) {
            if (std::chrono::steady_clock::now() >= deadline) {
                return Result::failure("Timed out waiting for " + std::to_string(data.size() - offset) +
                                       " byte(s)");
            }

            pollfd descriptor{.fd = fd_, .events = POLLIN, .revents = 0};
            const int ready = ::poll(&descriptor, 1, poll_timeout_ms(deadline));
            if (ready < 0) {
                if (errno == EINTR) {
                    continue;
                }
                return Result::failure(errno_message("Failed to wait for serial data"));
            }
            if (ready == 0) {
                return Result::failure("Timed out waiting for serial data");
            }

            const ssize_t received = ::read(fd_, data.data() + offset, data.size() - offset);
            if (received < 0) {
                if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                }
                return Result::failure(errno_message("Failed to read from serial port"));
            }
            if (received == 0) {
                return Result::failure("Serial port closed while reading");
            }
            offset += static_cast<size_t>(received);
        }
        return Result::success();
    }

    void ScsSerialPort::flush_input() const {
        if (is_open()) {
            ::ioctl(fd_, TCFLSH, TCIFLUSH);
        }
    }

} // namespace feetech_servo_ros2_control
