#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

#include "feetech_servo_ros2_control/scs_protocol.hpp"
#include "feetech_servo_ros2_control/scs_register_map.hpp"

namespace feetech_servo_ros2_control {
    namespace {

        std::vector<uint8_t> bytes(std::initializer_list<uint8_t> values) {
            return std::vector<uint8_t>(values);
        }

        TEST(ScsPacket, PingMatchesProtocolManual) {
            const auto packet =
                make_packet(0x01, kInstructionPing, std::span<const uint8_t>{});
            EXPECT_EQ(packet, bytes({0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}));
        }

        TEST(ScsPacket, ReadMatchesProtocolManual) {
            const std::array<uint8_t, 2> params{0x38, 0x02};
            const auto packet = make_packet(0x01, kInstructionRead, params);
            EXPECT_EQ(packet, bytes({0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE}));
        }

        TEST(ScsPacket, WriteMatchesProtocolManual) {
            const std::array<uint8_t, 7> params{0x2A, 0x01, 0xFF, 0x00, 0x00, 0x03, 0xE8};
            const auto packet = make_packet(0x01, kInstructionWrite, params);
            EXPECT_EQ(packet, bytes({0xFF, 0xFF, 0x01, 0x09, 0x03, 0x2A, 0x01, 0xFF, 0x00, 0x00, 0x03, 0xE8, 0xDD}));
        }

        TEST(ScsPacket, ChecksumWrapsAtOneByte) {
            const std::array<uint8_t, 3> params{0xFF, 0xFF, 0xFF};
            // id + length + instruction + params = 1 + 5 + 3 + 765 = 774 = 0x306, low byte 0x06.
            EXPECT_EQ(packet_checksum(1, 5, 3, params), static_cast<uint8_t>(~0x06));
        }

        TEST(ScsEncoding, WordsAreHighByteFirst) {
            std::array<uint8_t, 2> buffer{};
            encode_word(buffer.data(), 0x2305);
            EXPECT_EQ(buffer[0], 0x23);
            EXPECT_EQ(buffer[1], 0x05);
            EXPECT_EQ(decode_word(buffer.data()), 0x2305);
        }

        TEST(ScsEncoding, SignMagnitudeDecoding) {
            EXPECT_EQ(decode_sign_magnitude(0x0000, kSpeedSignBit), 0);
            EXPECT_EQ(decode_sign_magnitude(0x03E8, kSpeedSignBit), 1000);
            EXPECT_EQ(decode_sign_magnitude(0x83E8, kSpeedSignBit), -1000);
            EXPECT_EQ(decode_sign_magnitude(0x0400 | 500, kLoadSignBit), -500);
            EXPECT_EQ(decode_sign_magnitude(500, kLoadSignBit), 500);
        }

        TEST(ScsModel, KnownModels) {
            EXPECT_EQ(model_name(kModelSc215), "SC215");
            EXPECT_EQ(model_name(kModelSc225), "SC225");
            EXPECT_TRUE(is_supported_model(kModelSc215));
            EXPECT_TRUE(is_supported_model(kModelSc225));
            EXPECT_FALSE(is_supported_model(0x0309));
        }

        TEST(ScsPosition, TickToRadiansUsesMechanicalRange) {
            constexpr double kRangeDeg = 300.0;
            constexpr double kPi = 3.14159265358979323846;

            EXPECT_DOUBLE_EQ(tick_to_radians(kPositionMidpoint, kRangeDeg), 0.0);
            EXPECT_NEAR(tick_to_radians(kPositionTicks - 1, kRangeDeg),
                        (kPositionTicks - 1 - kPositionMidpoint) * kRangeDeg * kPi / 180.0 / kPositionTicks,
                        1e-12);
            EXPECT_NEAR(tick_to_radians(0, kRangeDeg),
                        -static_cast<double>(kPositionMidpoint) * kRangeDeg * kPi / 180.0 / kPositionTicks,
                        1e-12);
        }

        TEST(ScsPosition, RadiansToTickRoundTripsAndClamps) {
            constexpr double kRangeDeg = 300.0;
            constexpr double kPi = 3.14159265358979323846;

            EXPECT_EQ(radians_to_tick(0.0, kRangeDeg, 0, 1023), kPositionMidpoint);
            EXPECT_EQ(radians_to_tick(kPi / 2.0, kRangeDeg, 0, 1023), 818);
            EXPECT_EQ(radians_to_tick(tick_to_radians(700, kRangeDeg), kRangeDeg, 0, 1023), 700);

            EXPECT_EQ(radians_to_tick(-10.0, kRangeDeg, 0, 1023), 0);
            EXPECT_EQ(radians_to_tick(10.0, kRangeDeg, 0, 1023), 1023);
            EXPECT_EQ(radians_to_tick(-10.0, kRangeDeg, 100, 900), 100);
            EXPECT_EQ(radians_to_tick(10.0, kRangeDeg, 100, 900), 900);
        }

    } // namespace
} // namespace feetech_servo_ros2_control
