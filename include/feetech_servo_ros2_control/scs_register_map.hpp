#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string_view>

namespace feetech_servo_ros2_control {

    // SCSCL memory table (Feetech "SCS series, potentiometer" control table).
    // Addresses are the low address of each field. Multi-byte fields are
    // transmitted high byte first, matching SCServo_Linux's SCSCL (End = 1).
    inline constexpr uint8_t kRegisterFirmwareMajor = 0;
    inline constexpr uint8_t kRegisterFirmwareMinor = 1;
    inline constexpr uint8_t kRegisterEnd = 2;
    inline constexpr uint8_t kRegisterServoMajor = 3;
    inline constexpr uint8_t kRegisterServoMinor = 4;
    inline constexpr uint8_t kRegisterModel = kRegisterServoMajor; // 2 bytes: servo major/minor
    inline constexpr uint8_t kRegisterId = 5;
    inline constexpr uint8_t kRegisterBaudRate = 6;
    inline constexpr uint8_t kRegisterReturnDelay = 7;
    inline constexpr uint8_t kRegisterResponseLevel = 8;
    inline constexpr uint8_t kRegisterMinAngleLimit = 9; // 2 bytes, ticks
    inline constexpr uint8_t kRegisterMaxAngleLimit = 11; // 2 bytes, ticks
    inline constexpr uint8_t kRegisterMaxTemperature = 13; // degC
    inline constexpr uint8_t kRegisterMaxInputVoltage = 14; // 0.1 V
    inline constexpr uint8_t kRegisterMinInputVoltage = 15; // 0.1 V
    inline constexpr uint8_t kRegisterMaxTorque = 16; // 2 bytes, 0..1000
    inline constexpr uint8_t kRegisterUnloadingCondition = 19; // bitfield
    inline constexpr uint8_t kRegisterLedAlarmCondition = 20; // bitfield
    inline constexpr uint8_t kRegisterPCoefficient = 21;
    inline constexpr uint8_t kRegisterDCoefficient = 22;
    inline constexpr uint8_t kRegisterCwDeadZone = 26;
    inline constexpr uint8_t kRegisterCcwDeadZone = 27;
    inline constexpr uint8_t kRegisterProtectionTorque = 37; // percent of max torque
    inline constexpr uint8_t kRegisterProtectionTime = 38; // 40 ms units
    inline constexpr uint8_t kRegisterOverloadTorque = 39; // percent of max torque

    inline constexpr uint8_t kRegisterTorqueEnable = 40;
    inline constexpr uint8_t kRegisterGoalPosition = 42; // 2 bytes
    inline constexpr uint8_t kRegisterGoalTime = 44; // 2 bytes, ms
    inline constexpr uint8_t kRegisterGoalSpeed = 46; // 2 bytes, ticks/s
    inline constexpr uint8_t kRegisterEepromLock = 48;

    inline constexpr uint8_t kRegisterPresentPosition = 56; // 2 bytes
    inline constexpr uint8_t kRegisterPresentSpeed = 58; // 2 bytes, sign bit 15
    inline constexpr uint8_t kRegisterPresentLoad = 60; // 2 bytes, sign bit 10
    inline constexpr uint8_t kRegisterPresentVoltage = 62; // 0.1 V
    inline constexpr uint8_t kRegisterPresentTemperature = 63; // degC
    inline constexpr uint8_t kRegisterMoving = 66;

    // Number of bytes returned by a single sequential read of all feedback states:
    // position (2), speed (2), load (2), voltage (1), temperature (1).
    inline constexpr uint8_t kFeedbackLength = 8;

    inline constexpr uint8_t kInstructionPing = 0x01;
    inline constexpr uint8_t kInstructionRead = 0x02;
    inline constexpr uint8_t kInstructionWrite = 0x03;
    inline constexpr uint8_t kInstructionSyncWrite = 0x83;

    inline constexpr uint8_t kBroadcastId = 0xFE;
    inline constexpr uint8_t kMaxServoId = 253;

    // SCS potentiometer servos have a 10-bit absolute encoder.
    inline constexpr uint16_t kPositionTicks = 1024;
    inline constexpr uint16_t kPositionMidpoint = 511;

    inline constexpr uint8_t kSpeedSignBit = 15;
    inline constexpr uint8_t kLoadSignBit = 10;

    // Model identifier reported at kRegisterModel. SCS registers are high byte
    // first, so SC215 (major 5, minor 35) reads as 0x0523 and SC225
    // (major 9, minor 35) as 0x0923. The reference debug tool stores the same
    // values byte-swapped (0x2305/0x2309) because it reads them low byte first.
    inline constexpr uint16_t kModelSc215 = 0x0523;
    inline constexpr uint16_t kModelSc225 = 0x0923;

    inline std::string_view model_name(const uint16_t model) {
        switch (model) {
        case kModelSc215:
            return "SC215";
        case kModelSc225:
            return "SC225";
        default:
            return "unknown";
        }
    }

    inline bool is_supported_model(const uint16_t model) {
        return model == kModelSc215 || model == kModelSc225;
    }

    /// SCS multi-byte fields are high byte first.
    inline void encode_word(uint8_t* const out, const uint16_t value) {
        out[0] = static_cast<uint8_t>(value >> 8);
        out[1] = static_cast<uint8_t>(value & 0xFF);
    }

    inline uint16_t decode_word(const uint8_t* const in) {
        return static_cast<uint16_t>((static_cast<uint16_t>(in[0]) << 8) | in[1]);
    }

    /// Decode a sign-magnitude word (used for present speed and present load).
    inline int decode_sign_magnitude(const uint16_t raw, const uint8_t sign_bit) {
        const bool negative = ((raw >> sign_bit) & 1U) != 0;
        const int magnitude = static_cast<int>(raw & ((1U << sign_bit) - 1U));
        return negative ? -magnitude : magnitude;
    }

    /// A servo covers `range_deg` mechanical degrees over its 0..1023 tick range.
    /// The driver converts ticks to radians with this scale, so commanded joint
    /// angles match the physical servo angle.
    inline double radians_per_tick(const double range_deg) {
        constexpr double kPi = 3.14159265358979323846;
        return range_deg * kPi / 180.0 / static_cast<double>(kPositionTicks);
    }

    /// Degrees relative to the servo center (tick 511 = 0 degree).
    inline double degrees_per_tick(const double range_deg) {
        return range_deg / static_cast<double>(kPositionTicks);
    }

    inline double tick_to_degrees(const uint16_t tick, const double range_deg) {
        return (static_cast<double>(tick) - static_cast<double>(kPositionMidpoint)) *
            degrees_per_tick(range_deg);
    }

    inline uint16_t degrees_to_tick(const double degrees, const double range_deg) {
        const auto tick = static_cast<long>(std::lround(degrees / degrees_per_tick(range_deg))) +
            static_cast<long>(kPositionMidpoint);
        return static_cast<uint16_t>(std::clamp<long>(tick, 0, static_cast<long>(kPositionTicks) - 1));
    }

    inline double tick_to_radians(const uint16_t tick, const double range_deg) {
        return (static_cast<double>(tick) - static_cast<double>(kPositionMidpoint)) *
            radians_per_tick(range_deg);
    }

    inline uint16_t radians_to_tick(const double radians, const double range_deg,
                                    const uint16_t min_tick, const uint16_t max_tick) {
        const auto tick = static_cast<long>(std::lround(radians / radians_per_tick(range_deg))) +
            static_cast<long>(kPositionMidpoint);
        const long lower = std::max<long>(0, min_tick);
        const long upper = std::min<long>(static_cast<long>(kPositionTicks) - 1, max_tick);
        return static_cast<uint16_t>(std::clamp(tick, lower, upper));
    }

} // namespace feetech_servo_ros2_control
