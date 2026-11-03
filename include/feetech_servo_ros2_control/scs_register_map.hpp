#pragma once

#include <algorithm>
#include <cmath>
#include <string_view>

namespace feetech_servo_ros2_control {

    // SCSCL 内存表（飞特“SCS 电位器版”控制表）。
    // 地址为字段起始地址；多字节字段高字节在前，与 SCServo_Linux 的 SCSCL（End = 1）一致。
    inline constexpr uint8_t kRegisterFirmwareMajor = 0;
    inline constexpr uint8_t kRegisterFirmwareMinor = 1;
    inline constexpr uint8_t kRegisterEnd = 2;
    inline constexpr uint8_t kRegisterServoMajor = 3;
    inline constexpr uint8_t kRegisterServoMinor = 4;
    inline constexpr uint8_t kRegisterModel = kRegisterServoMajor; // 2 字节：舵机主/次版本号
    inline constexpr uint8_t kRegisterId = 5;
    inline constexpr uint8_t kRegisterBaudRate = 6;
    inline constexpr uint8_t kRegisterReturnDelay = 7;
    inline constexpr uint8_t kRegisterResponseLevel = 8;
    inline constexpr uint8_t kRegisterMinAngleLimit = 9; // 2 字节，单位 tick
    inline constexpr uint8_t kRegisterMaxAngleLimit = 11; // 2 字节，单位 tick
    inline constexpr uint8_t kRegisterMaxTemperature = 13; // 单位 °C
    inline constexpr uint8_t kRegisterMaxInputVoltage = 14; // 单位 0.1 V
    inline constexpr uint8_t kRegisterMinInputVoltage = 15; // 单位 0.1 V
    inline constexpr uint8_t kRegisterMaxTorque = 16; // 2 字节，0..1000
    inline constexpr uint8_t kRegisterUnloadingCondition = 19; // 位域
    inline constexpr uint8_t kRegisterLedAlarmCondition = 20; // 位域
    inline constexpr uint8_t kRegisterPCoefficient = 21;
    inline constexpr uint8_t kRegisterDCoefficient = 22;
    inline constexpr uint8_t kRegisterCwDeadZone = 26;
    inline constexpr uint8_t kRegisterCcwDeadZone = 27;
    inline constexpr uint8_t kRegisterProtectionTorque = 37; // 最大扭矩百分比
    inline constexpr uint8_t kRegisterProtectionTime = 38; // 单位 40 ms
    inline constexpr uint8_t kRegisterOverloadTorque = 39; // 最大扭矩百分比

    inline constexpr uint8_t kRegisterTorqueEnable = 40;
    inline constexpr uint8_t kRegisterGoalPosition = 42; // 2 字节
    inline constexpr uint8_t kRegisterGoalTime = 44; // 2 字节，单位 ms
    inline constexpr uint8_t kRegisterGoalSpeed = 46; // 2 字节，单位 tick/s
    inline constexpr uint8_t kRegisterEepromLock = 48;

    inline constexpr uint8_t kRegisterPresentPosition = 56; // 2 字节
    inline constexpr uint8_t kRegisterPresentSpeed = 58; // 2 字节，符号位 15
    inline constexpr uint8_t kRegisterPresentLoad = 60; // 2 字节，符号位 10
    inline constexpr uint8_t kRegisterPresentVoltage = 62; // 单位 0.1 V
    inline constexpr uint8_t kRegisterPresentTemperature = 63; // 单位 °C
    inline constexpr uint8_t kRegisterMoving = 66;
    inline constexpr uint8_t kRegisterPresentCurrent = 69; // 2 字节，符号位 15

    // 一次顺序读取全部反馈状态返回的字节数（地址 0x38..0x46）：
    // 位置（2）、速度（2）、负载（2）、电压（1）、温度（1）、
    // 异步写标志（1）、状态（1）、移动标志（1）、保留（2）、电流（2）。
    inline constexpr uint8_t kFeedbackLength = 15;

    inline constexpr uint8_t kInstructionPing = 0x01;
    inline constexpr uint8_t kInstructionRead = 0x02;
    inline constexpr uint8_t kInstructionWrite = 0x03;
    inline constexpr uint8_t kInstructionSyncWrite = 0x83;

    inline constexpr uint8_t kBroadcastId = 0xFE;
    inline constexpr uint8_t kMaxServoId = 253;

    // SCS 电位器版舵机使用 10 位绝对编码器。
    inline constexpr uint16_t kPositionTicks = 1024;
    inline constexpr uint16_t kPositionMidpoint = 511;

    inline constexpr uint8_t kSpeedSignBit = 15;
    inline constexpr uint8_t kLoadSignBit = 10;
    inline constexpr uint8_t kCurrentSignBit = 15;

    // 电流寄存器每个 LSB 对应的安培数（SCS 系列为 6.5 mA）。
    inline constexpr double kCurrentAmpsPerUnit = 0.0065;

    // kRegisterModel 上报告的型号标识。SCS 寄存器高字节在前，因此：
    // SC215（主版本 5、次版本 35）读回 0x0523，SC225（主版本 9、次版本 35）读回 0x0923。
    // 参考调试工具按低字节在前读取，所以其中保存的是字节交换后的 0x2305/0x2309。
    inline constexpr uint16_t kModelSc215 = 0x0523;
    inline constexpr uint16_t kModelSc225 = 0x0923;

    /// @brief 将型号编号转换为可读名称。
    /// @param model 型号编号，例如 kModelSc215 / kModelSc225。
    /// @return 型号名称；未知型号返回 "unknown"。
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

    /// @brief 判断型号是否受支持（SC215/SC225）。
    inline bool is_supported_model(const uint16_t model) {
        return model == kModelSc215 || model == kModelSc225;
    }

    /// @brief 将 16 位值按 SCS 字节序（高字节在前）写入缓冲区。
    /// @param out 至少 2 字节的输出缓冲区。
    /// @param value 待写入的值。
    inline void encode_word(uint8_t* const out, const uint16_t value) {
        out[0] = static_cast<uint8_t>(value >> 8);
        out[1] = static_cast<uint8_t>(value & 0xFF);
    }

    /// @brief 按 SCS 字节序（高字节在前）解析 16 位值。
    /// @param in 至少 2 字节的输入缓冲区。
    inline uint16_t decode_word(const uint8_t* const in) {
        return static_cast<uint16_t>((static_cast<uint16_t>(in[0]) << 8) | in[1]);
    }

    /// @brief 解码符号-幅值表示的数据（用于当前速度和当前负载）。
    /// @param raw 原始 16 位值。
    /// @param sign_bit 符号位的位置，速度用 15，负载用 10。
    /// @return 带符号的幅值。
    inline int decode_sign_magnitude(const uint16_t raw, const uint8_t sign_bit) {
        const bool negative = ((raw >> sign_bit) & 1U) != 0;
        const int magnitude = static_cast<int>(raw & ((1U << sign_bit) - 1U));
        return negative ? -magnitude : magnitude;
    }

    /// @brief 计算每个 tick 对应的弧度。
    /// @param range_deg 舵机 0..1023 tick 对应的机械行程（度）。
    inline double radians_per_tick(const double range_deg) {
        constexpr double kPi = 3.14159265358979323846;
        return range_deg * kPi / 180.0 / static_cast<double>(kPositionTicks);
    }

    /// @brief 计算每个 tick 对应的角度（度）。
    /// @param range_deg 舵机 0..1023 tick 对应的机械行程（度）。
    inline double degrees_per_tick(const double range_deg) {
        return range_deg / static_cast<double>(kPositionTicks);
    }

    /// @brief 将 tick 转换为相对中位的角度（度）。
    /// @param tick 0..1023 的位置值。
    /// @param range_deg 舵机机械行程（度）。
    inline double tick_to_degrees(const uint16_t tick, const double range_deg) {
        return (static_cast<double>(tick) - static_cast<double>(kPositionMidpoint)) *
            degrees_per_tick(range_deg);
    }

    /// @brief 将相对中位的角度（度）转换为 tick，并钳位到 0..1023。
    /// @param degrees 相对中位的角度，0 度对应 tick 511。
    /// @param range_deg 舵机机械行程（度）。
    inline uint16_t degrees_to_tick(const double degrees, const double range_deg) {
        const auto tick = static_cast<long>(std::lround(degrees / degrees_per_tick(range_deg))) +
            static_cast<long>(kPositionMidpoint);
        return static_cast<uint16_t>(std::clamp<long>(tick, 0, static_cast<long>(kPositionTicks) - 1));
    }

    /// @brief 将 tick 转换为相对中位的弧度。
    /// @param tick 0..1023 的位置值。
    /// @param range_deg 舵机机械行程（度）。
    inline double tick_to_radians(const uint16_t tick, const double range_deg) {
        return (static_cast<double>(tick) - static_cast<double>(kPositionMidpoint)) *
            radians_per_tick(range_deg);
    }

    /// @brief 将相对中位的弧度转换为 tick，并钳位到 [0, 1023] 与 [min_tick, max_tick]。
    /// @param radians 相对中位的弧度，0 弧度对应 tick 511。
    /// @param range_deg 舵机机械行程（度）。
    /// @param min_tick 允许的最小 tick（关节下限）。
    /// @param max_tick 允许的最大 tick（关节上限）。
    inline uint16_t radians_to_tick(const double radians, const double range_deg,
                                    const uint16_t min_tick, const uint16_t max_tick) {
        const auto tick = static_cast<long>(std::lround(radians / radians_per_tick(range_deg))) +
            static_cast<long>(kPositionMidpoint);
        const long lower = std::max<long>(0, min_tick);
        const long upper = std::min<long>(static_cast<long>(kPositionTicks) - 1, max_tick);
        return static_cast<uint16_t>(std::clamp(tick, lower, upper));
    }

} // namespace feetech_servo_ros2_control
