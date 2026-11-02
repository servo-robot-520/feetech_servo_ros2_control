#pragma once

#include <chrono>
#include <cstdint>
#include <span>
#include <vector>

#include "feetech_servo_ros2_control/result.hpp"
#include "feetech_servo_ros2_control/scs_serial_port.hpp"

namespace feetech_servo_ros2_control {

    /// Build a Feetech protocol packet:
    ///   FF FF ID LENGTH INSTRUCTION PARAMS... CHECKSUM
    /// LENGTH counts instruction, parameters and checksum.
    std::vector<uint8_t> make_packet(uint8_t id, uint8_t instruction,
                                     std::span<const uint8_t> params);

    /// Compute the low-byte checksum used by the protocol.
    uint8_t packet_checksum(uint8_t id, uint8_t length, uint8_t instruction,
                            std::span<const uint8_t> params);

    /// SCS (SCSCL) protocol client. One instance drives one serial bus.
    class ScsProtocol {
    public:
        ScsProtocol(ScsSerialPort& port, std::chrono::milliseconds response_timeout);

        Result ping(uint8_t id);

        Result read_registers(uint8_t id, uint8_t address, std::span<uint8_t> data);

        Result write_registers(uint8_t id, uint8_t address, std::span<const uint8_t> data);

        /// Write goal position/time/speed to a single servo.
        Result write_goal_position(uint8_t id, uint16_t position, uint16_t time_ms,
                                   uint16_t speed);

        /// Broadcast one SYNC WRITE for goal position (address 0x2A) covering
        /// position, time and speed for every servo on the bus.
        Result sync_write_goal_positions(std::span<const uint8_t> ids,
                                         std::span<const uint16_t> positions, uint16_t time_ms,
                                         std::span<const uint16_t> speeds);

        Result set_torque_enabled(uint8_t id, bool enabled);
        Result set_eeprom_locked(uint8_t id, bool locked);

        /// Read a high-byte-first 16-bit register.
        Expected<uint16_t> read_word(uint8_t id, uint8_t address);

        /// 16-bit model/firmware identifier at kRegisterModel.
        Expected<uint16_t> read_model(uint8_t id);

    private:
        Result transact(const std::vector<uint8_t>& packet, uint8_t expected_id,
                        std::span<uint8_t> payload);

        ScsSerialPort& port_;
        std::chrono::milliseconds response_timeout_;
    };

} // namespace feetech_servo_ros2_control
