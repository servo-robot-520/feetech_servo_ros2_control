#pragma once

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "feetech_servo_ros2_control/scs_register_map.hpp"

namespace feetech_servo_ros2_control::test {

    /// @brief 轮询谓词，直到其为真或超时。
    /// @param predicate 待轮询的条件。
    /// @param timeout 最长等待时间。
    /// @return 谓词最终是否为真。
    template <typename Predicate>
    bool wait_for(Predicate predicate, const std::chrono::milliseconds timeout = std::chrono::milliseconds(200)) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (predicate()) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return predicate();
    }

    /// @brief 基于伪终端的极简 SCS 总线模拟器。
    ///
    /// 驱动打开 pty 从端，本类在主端应答，实现 PING、READ、WRITE 和 SYNC WRITE，
    /// 可模拟一组舵机 id。
    class FakeScsBus {
    public:
        explicit FakeScsBus(const std::vector<uint8_t>& ids, const uint16_t model = kModelSc215) {
            master_fd_ = ::posix_openpt(O_RDWR | O_NOCTTY);
            if (master_fd_ < 0) {
                throw std::runtime_error("posix_openpt failed");
            }
            if (::grantpt(master_fd_) != 0 || ::unlockpt(master_fd_) != 0) {
                ::close(master_fd_);
                throw std::runtime_error("grantpt/unlockpt failed");
            }
            std::array<char, 128> path{};
            if (::ptsname_r(master_fd_, path.data(), path.size()) != 0) {
                ::close(master_fd_);
                throw std::runtime_error("ptsname_r failed");
            }
            slave_path_ = path.data();
            ::fcntl(master_fd_, F_SETFL, O_NONBLOCK);

            for (const auto id : ids) {
                auto& servo = servos_[id];
                servo.registers.fill(0);
                servo.registers[kRegisterModel] = static_cast<uint8_t>(model >> 8);
                servo.registers[kRegisterModel + 1] = static_cast<uint8_t>(model & 0xFF);
                servo.registers[kRegisterId] = id;
                servo.registers[kRegisterResponseLevel] = 1;
                servo.registers[kRegisterMinAngleLimit] = 0x00;
                servo.registers[kRegisterMinAngleLimit + 1] = 0x00;
                servo.registers[kRegisterMaxAngleLimit] = 0x03;
                servo.registers[kRegisterMaxAngleLimit + 1] = 0xFF;
                servo.registers[kRegisterFirmwareMajor] = 3;
                servo.registers[kRegisterFirmwareMinor] = 24;
                servo.registers[kRegisterEnd] = 1;
                servo.registers[kRegisterMaxTemperature] = 80;
                servo.registers[kRegisterMaxInputVoltage] = 90;
                servo.registers[kRegisterMinInputVoltage] = 45;
                servo.registers[kRegisterMaxTorque] = 0x03;
                servo.registers[kRegisterMaxTorque + 1] = 0xE8;
                servo.registers[kRegisterUnloadingCondition] = 32;
                servo.registers[kRegisterLedAlarmCondition] = 37;
                servo.registers[kRegisterPCoefficient] = 15;
                servo.registers[kRegisterDCoefficient] = 15;
                servo.registers[kRegisterCwDeadZone] = 1;
                servo.registers[kRegisterCcwDeadZone] = 1;
                servo.registers[kRegisterProtectionTorque] = 20;
                servo.registers[kRegisterProtectionTime] = 100;
                servo.registers[kRegisterOverloadTorque] = 80;
                servo.registers[kRegisterEepromLock] = 1;
                encode_word(&servo.registers[kRegisterPresentPosition], kPositionMidpoint);
                servo.registers[kRegisterPresentVoltage] = 74;
                servo.registers[kRegisterPresentTemperature] = 30;
            }

            thread_ = std::thread([this] { run(); });
        }

        ~FakeScsBus() {
            stop_.store(true);
            if (thread_.joinable()) {
                thread_.join();
            }
            ::close(master_fd_);
        }

        FakeScsBus(const FakeScsBus&) = delete;
        FakeScsBus& operator=(const FakeScsBus&) = delete;

        const std::string& slave_path() const {
            return slave_path_;
        }

        /// @brief 设置模拟舵机的当前位置。
        void set_present_position(const uint8_t id, const uint16_t ticks) {
            std::lock_guard<std::mutex> lock(mutex_);
            encode_word(&servos_.at(id).registers[kRegisterPresentPosition], ticks);
        }

        /// @brief 读取模拟舵机的当前位置。
        uint16_t present_position(const uint8_t id) const {
            std::lock_guard<std::mutex> lock(mutex_);
            return decode_word(&servos_.at(id).registers[kRegisterPresentPosition]);
        }

        /// @brief 设置模拟舵机的当前速度原始值（符号位 15）。
        void set_present_speed(const uint8_t id, const uint16_t raw) {
            std::lock_guard<std::mutex> lock(mutex_);
            encode_word(&servos_.at(id).registers[kRegisterPresentSpeed], raw);
        }

        /// @brief 设置模拟舵机的当前负载原始值（符号位 10）。
        void set_present_load(const uint8_t id, const uint16_t raw) {
            std::lock_guard<std::mutex> lock(mutex_);
            encode_word(&servos_.at(id).registers[kRegisterPresentLoad], raw);
        }

        /// @brief 设置模拟舵机的当前电流原始值（单位 6.5 mA）。
        void set_present_current(const uint8_t id, const uint16_t raw) {
            std::lock_guard<std::mutex> lock(mutex_);
            encode_word(&servos_.at(id).registers[kRegisterPresentCurrent], raw);
        }

        /// @brief 查询模拟舵机的扭矩开关状态。
        bool torque_enabled(const uint8_t id) const {
            std::lock_guard<std::mutex> lock(mutex_);
            return servos_.at(id).registers[kRegisterTorqueEnable] == 1;
        }

        /// @brief 读取模拟舵机某个寄存器的原始值。
        uint8_t register_value(const uint8_t id, const uint8_t address) const {
            std::lock_guard<std::mutex> lock(mutex_);
            return servos_.at(id).registers.at(address);
        }

        /// @brief 丢弃下一次应答，用于测试超时与重新同步。
        void drop_next_response() {
            drop_next_response_.store(true);
        }

        /// @brief 设置每个应答前附加的噪声字节，用于测试帧头重新同步。
        void set_response_prefix(const std::span<const uint8_t> prefix) {
            std::lock_guard<std::mutex> lock(mutex_);
            response_prefix_.assign(prefix.begin(), prefix.end());
        }

        /// @brief 直接向驱动侧写入原始字节（用于注入总线噪声）。
        void send_raw(const std::span<const uint8_t> bytes) {
            size_t offset = 0;
            while (offset < bytes.size()) {
                const ssize_t written = ::write(master_fd_, bytes.data() + offset, bytes.size() - offset);
                if (written > 0) {
                    offset += static_cast<size_t>(written);
                }
                else if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                    pollfd descriptor{master_fd_, POLLOUT, 0};
                    ::poll(&descriptor, 1, 10);
                }
                else if (written < 0 && errno == EINTR) {
                    continue;
                }
                else {
                    return;
                }
            }
        }

    private:
        struct Servo {
            std::array<uint8_t, 128> registers{};
        };

        void run() {
            std::vector<uint8_t> buffer;
            while (!stop_.load()) {
                pollfd descriptor{master_fd_, POLLIN, 0};
                const int ready = ::poll(&descriptor, 1, 5);
                if (ready <= 0) {
                    continue;
                }
                std::array<uint8_t, 256> chunk{};
                const ssize_t received = ::read(master_fd_, chunk.data(), chunk.size());
                if (received <= 0) {
                    continue;
                }
                buffer.insert(buffer.end(), chunk.begin(), chunk.begin() + received);
                process(buffer);
            }
        }

        void process(std::vector<uint8_t>& buffer) {
            while (true) {
                size_t header = 0;
                while (header + 1 < buffer.size() &&
                       !(buffer[header] == 0xFF && buffer[header + 1] == 0xFF)) {
                    ++header;
                }
                if (header > 0) {
                    buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(header));
                }
                if (buffer.size() < 4) {
                    return;
                }
                const uint8_t length = buffer[3];
                const size_t total = 4 + length;
                if (length < 2) {
                    buffer.erase(buffer.begin());
                    continue;
                }
                if (buffer.size() < total) {
                    return;
                }

                const uint8_t id = buffer[2];
                uint8_t sum = static_cast<uint8_t>(id + length);
                for (size_t i = 4; i + 1 < total; ++i) {
                    sum = static_cast<uint8_t>(sum + buffer[i]);
                }
                if (static_cast<uint8_t>(~sum) != buffer[total - 1]) {
                    buffer.erase(buffer.begin());
                    continue;
                }

                handle_packet(id, buffer[4], std::span<const uint8_t>(buffer.data() + 5, length - 2));
                buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(total));
            }
        }

        void handle_packet(const uint8_t id, const uint8_t instruction,
                           const std::span<const uint8_t> params) {
            switch (instruction) {
            case kInstructionPing:
                if (servos_.find(id) != servos_.end()) {
                    respond(id, std::span<const uint8_t>{});
                }
                break;
            case kInstructionRead: {
                if (params.size() != 2) {
                    return;
                }
                const uint8_t address = params[0];
                const uint8_t count = params[1];
                std::vector<uint8_t> payload(count, 0);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    const auto servo = servos_.find(id);
                    if (servo == servos_.end()) {
                        return;
                    }
                    for (size_t i = 0; i < count; ++i) {
                        const size_t register_index = address + i;
                        if (register_index >= servo->second.registers.size()) {
                            return;
                        }
                        payload[i] = servo->second.registers[register_index];
                    }
                }
                respond(id, payload);
                break;
            }
            case kInstructionWrite: {
                if (params.size() < 2) {
                    return;
                }
                write_registers(id, params[0], params.subspan(1));
                respond(id, std::span<const uint8_t>{});
                break;
            }
            case kInstructionSyncWrite: {
                if (params.size() < 2) {
                    return;
                }
                const uint8_t address = params[0];
                const uint8_t data_length = params[1];
                if (data_length == 0) {
                    return;
                }
                size_t offset = 2;
                while (offset + 1 + data_length <= params.size()) {
                    write_registers(params[offset], address,
                                    params.subspan(offset + 1, data_length));
                    offset += 1 + data_length;
                }
                break;
            }
            default:
                break;
            }
        }

        void write_registers(const uint8_t id, const uint8_t address,
                             const std::span<const uint8_t> data) {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto servo = servos_.find(id);
            if (servo == servos_.end()) {
                return;
            }
            auto& registers = servo->second.registers;
            for (size_t i = 0; i < data.size(); ++i) {
                if (address + i < registers.size()) {
                    registers[address + i] = data[i];
                }
            }
            // 把目标位置镜像到当前位置，并回显命令速度，使测试得到确定性的闭环结果。
            if (address <= kRegisterGoalPosition &&
                address + data.size() > kRegisterGoalPosition + 1) {
                registers[kRegisterPresentPosition] = registers[kRegisterGoalPosition];
                registers[kRegisterPresentPosition + 1] = registers[kRegisterGoalPosition + 1];
            }
            if (address <= kRegisterGoalSpeed && address + data.size() > kRegisterGoalSpeed + 1) {
                registers[kRegisterPresentSpeed] = registers[kRegisterGoalSpeed];
                registers[kRegisterPresentSpeed + 1] = registers[kRegisterGoalSpeed + 1];
            }
        }

        void respond(const uint8_t id, const std::span<const uint8_t> payload) {
            if (drop_next_response_.exchange(false)) {
                return;
            }
            std::vector<uint8_t> prefix;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                prefix = response_prefix_;
            }
            if (!prefix.empty()) {
                send_raw(prefix);
            }
            const uint8_t length = static_cast<uint8_t>(payload.size() + 2);
            std::vector<uint8_t> packet;
            packet.reserve(payload.size() + 6);
            packet.push_back(0xFF);
            packet.push_back(0xFF);
            packet.push_back(id);
            packet.push_back(length);
            packet.push_back(0); // 状态字节
            packet.insert(packet.end(), payload.begin(), payload.end());
            uint8_t sum = static_cast<uint8_t>(id + length + 0);
            for (const auto byte : payload) {
                sum = static_cast<uint8_t>(sum + byte);
            }
            packet.push_back(static_cast<uint8_t>(~sum));
            send_raw(packet);
        }

        int master_fd_{-1};
        std::string slave_path_;
        std::unordered_map<uint8_t, Servo> servos_;
        mutable std::mutex mutex_;
        std::vector<uint8_t> response_prefix_;
        std::atomic<bool> stop_{false};
        std::atomic<bool> drop_next_response_{false};
        std::thread thread_;
    };

} // namespace feetech_servo_ros2_control::test
