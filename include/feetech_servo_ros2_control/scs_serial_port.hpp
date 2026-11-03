#pragma once

#include <chrono>
#include <cstdint>
#include <span>
#include <string>

#include "feetech_servo_ros2_control/result.hpp"

namespace feetech_servo_ros2_control {

    /// @brief 飞特 TTL 总线舵机的 Linux 串口传输层（8N1、原始模式）。
    ///
    /// 串口以非阻塞方式打开，每次读取都带明确超时，舵机无应答时不会阻塞控制循环。
    class ScsSerialPort {
    public:
        ScsSerialPort() = default;
        ~ScsSerialPort();

        ScsSerialPort(const ScsSerialPort&) = delete;
        ScsSerialPort& operator=(const ScsSerialPort&) = delete;

        /// @brief 打开并配置串口，支持任意波特率。
        /// @param device 串口设备路径，例如 /dev/ttyACM0。
        /// @param baud_rate 波特率，例如 1000000。
        /// @return 打开或配置失败时返回错误信息。
        Result open(const std::string& device, int baud_rate);

        /// @brief 关闭串口；未打开时为空操作。
        void close();

        /// @brief 串口是否已打开。
        bool is_open() const {
            return fd_ >= 0;
        }

        /// @brief 写入全部字节。
        /// @param data 待发送数据。
        /// @return 写入失败或超时时返回错误信息。
        Result write(std::span<const uint8_t> data);

        /// @brief 在超时时间内读满指定长度的数据。
        /// @param data 接收缓冲区。
        /// @param timeout 单次调用允许等待的最长时间。
        /// @return 超时、设备关闭或读取失败时返回错误信息。
        Result read_exact(std::span<uint8_t> data, std::chrono::milliseconds timeout);

        /// @brief 清空输入缓冲区，丢弃上一次超时后残留的字节。
        void flush_input();

    private:
        int fd_{-1};
    };

} // namespace feetech_servo_ros2_control
