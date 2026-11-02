#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <vector>

#include "fake_scs_servo.hpp"
#include "feetech_servo_ros2_control/scs_protocol.hpp"
#include "feetech_servo_ros2_control/scs_register_map.hpp"
#include "feetech_servo_ros2_control/scs_serial_port.hpp"

namespace feetech_servo_ros2_control {
    namespace {

        using test::FakeScsBus;

        TEST(ScsSerialPort, OpensPty) {
            FakeScsBus bus({1});
            ScsSerialPort port;
            ASSERT_TRUE(port.open(bus.slave_path(), 1000000));
            EXPECT_TRUE(port.is_open());
            port.close();
            EXPECT_FALSE(port.is_open());
        }

        TEST(ScsSerialPort, TimesOutWithoutData) {
            FakeScsBus bus({1});
            ScsSerialPort port;
            ASSERT_TRUE(port.open(bus.slave_path(), 1000000));
            std::array<uint8_t, 1> byte{};
            const auto result = port.read_exact(byte, std::chrono::milliseconds(10));
            EXPECT_FALSE(result);
        }

        TEST(ScsProtocol, PingReadAndWrite) {
            FakeScsBus bus({1});
            ScsSerialPort port;
            ASSERT_TRUE(port.open(bus.slave_path(), 1000000));
            ScsProtocol protocol(port, std::chrono::milliseconds(50));

            EXPECT_TRUE(protocol.ping(1));
            EXPECT_FALSE(protocol.ping(2));

            const auto model = protocol.read_model(1);
            ASSERT_TRUE(model);
            EXPECT_EQ(model.value(), kModelSc215);

            EXPECT_TRUE(protocol.set_torque_enabled(1, true));
            EXPECT_TRUE(bus.torque_enabled(1));

            EXPECT_TRUE(protocol.write_goal_position(1, 700, 0, 500));
            EXPECT_EQ(bus.present_position(1), 700);
        }

        TEST(ScsProtocol, SyncWriteUpdatesAllServos) {
            FakeScsBus bus({1, 2});
            ScsSerialPort port;
            ASSERT_TRUE(port.open(bus.slave_path(), 1000000));
            ScsProtocol protocol(port, std::chrono::milliseconds(50));

            const std::vector<uint8_t> ids{1, 2};
            const std::vector<uint16_t> positions{100, 900};
            const std::vector<uint16_t> speeds{500, 600};
            ASSERT_TRUE(protocol.sync_write_goal_positions(ids, positions, 0, speeds));
            // SYNC WRITE has no response: wait for the fake bus to consume the frame.
            ASSERT_TRUE(test::wait_for([&] {
                return bus.present_position(1) == 100 && bus.present_position(2) == 900;
            }));
            EXPECT_EQ(bus.present_position(1), 100);
            EXPECT_EQ(bus.present_position(2), 900);
        }

        TEST(ScsProtocol, RecoversAfterDroppedResponse) {
            FakeScsBus bus({1});
            ScsSerialPort port;
            ASSERT_TRUE(port.open(bus.slave_path(), 1000000));
            ScsProtocol protocol(port, std::chrono::milliseconds(30));

            bus.drop_next_response();
            EXPECT_FALSE(protocol.read_model(1));

            const auto model = protocol.read_model(1);
            ASSERT_TRUE(model);
            EXPECT_EQ(model.value(), kModelSc215);
        }

        TEST(ScsProtocol, ResynchronisesOnResponsePrefix) {
            FakeScsBus bus({1});
            ScsSerialPort port;
            ASSERT_TRUE(port.open(bus.slave_path(), 1000000));
            ScsProtocol protocol(port, std::chrono::milliseconds(50));

            const std::array<uint8_t, 2> noise{0x00, 0xAA};
            bus.set_response_prefix(noise);

            const auto model = protocol.read_model(1);
            ASSERT_TRUE(model);
            EXPECT_EQ(model.value(), kModelSc215);
        }

        TEST(ScsProtocol, RejectsMismatchedSyncWriteSizes) {
            FakeScsBus bus({1, 2});
            ScsSerialPort port;
            ASSERT_TRUE(port.open(bus.slave_path(), 1000000));
            ScsProtocol protocol(port, std::chrono::milliseconds(50));

            const std::vector<uint8_t> ids{1, 2};
            const std::vector<uint16_t> positions{100};
            const std::vector<uint16_t> speeds{500, 600};
            EXPECT_FALSE(protocol.sync_write_goal_positions(ids, positions, 0, speeds));
        }

    } // namespace
} // namespace feetech_servo_ros2_control
