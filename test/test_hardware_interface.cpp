#include <gtest/gtest.h>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "fake_scs_servo.hpp"
#include "feetech_servo_ros2_control/feetech_servo_hardware_interface.hpp"
#include "feetech_servo_ros2_control/scs_register_map.hpp"

namespace feetech_servo_ros2_control {
    namespace {

        using test::FakeScsBus;

        /// @brief 构造带有全部支持接口的测试关节。
        hardware_interface::ComponentInfo make_joint(const uint8_t id) {
            hardware_interface::ComponentInfo joint;
            joint.name = "joint_" + std::to_string(id);
            joint.parameters["id"] = std::to_string(id);
            for (const char* state_interface :
                 {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY, "load", "voltage",
                  "temperature", "current"}) {
                hardware_interface::InterfaceInfo info;
                info.name = state_interface;
                joint.state_interfaces.push_back(info);
            }
            hardware_interface::InterfaceInfo command;
            command.name = hardware_interface::HW_IF_POSITION;
            joint.command_interfaces.push_back(command);
            return joint;
        }

        /// @brief 构造指向指定串口和舵机 id 的测试硬件信息。
        hardware_interface::HardwareInfo make_hardware_info(const std::string& port,
                                                            const std::vector<uint8_t>& ids) {
            hardware_interface::HardwareInfo info;
            info.name = "scs_bus";
            info.type = "system";
            info.hardware_class_type = "feetech_servo_ros2_control/FeetechServoHardwareInterface";
            info.hardware_parameters["usb_port"] = port;
            info.hardware_parameters["baud_rate"] = "1000000";
            info.hardware_parameters["range_deg"] = "300";
            for (const auto id : ids) {
                info.joints.push_back(make_joint(id));
            }
            return info;
        }

        /// @brief 按驱动约定计算 tick 对应的弧度，用于校验读数。
        double expected_tick_radians(const uint16_t tick, const double range_deg) {
            return (static_cast<double>(tick) - 511.0) * range_deg * 3.14159265358979323846 / 180.0 / 1024.0;
        }

        class HardwareInterfaceTest : public ::testing::Test {
        protected:
            void SetUp() override {
                bus_ = std::make_unique<FakeScsBus>(std::vector<uint8_t>{1, 2});
            }

            std::unique_ptr<FakeScsBus> bus_;
        };

        TEST_F(HardwareInterfaceTest, InitialisesAndConfiguresEeprom) {
            auto info = make_hardware_info(bus_->slave_path(), {1, 2});
            for (auto& joint : info.joints) {
                joint.parameters["p_coefficient"] = "20";
            }

            FeetechServoHardwareInterface hardware;
            ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);

            EXPECT_EQ(bus_->register_value(1, kRegisterPCoefficient), 20);
            EXPECT_EQ(bus_->register_value(2, kRegisterPCoefficient), 20);
            EXPECT_EQ(bus_->register_value(1, kRegisterEepromLock), 1);
            EXPECT_FALSE(bus_->torque_enabled(1));
            EXPECT_FALSE(bus_->torque_enabled(2));
        }

        TEST_F(HardwareInterfaceTest, ExportsDeclaredInterfaces) {
            auto info = make_hardware_info(bus_->slave_path(), {1, 2});
            FeetechServoHardwareInterface hardware;
            ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);

            const auto states = hardware.export_state_interfaces();
            const auto commands = hardware.export_command_interfaces();
            EXPECT_EQ(states.size(), 12U);
            EXPECT_EQ(commands.size(), 2U);
            EXPECT_EQ(states[0].get_name(), "joint_1/position");
            EXPECT_EQ(states[4].get_name(), "joint_1/temperature");
            EXPECT_EQ(states[5].get_name(), "joint_1/current");
            EXPECT_EQ(commands[0].get_name(), "joint_1/position");
        }

        TEST_F(HardwareInterfaceTest, FullLifecycle) {
            auto info = make_hardware_info(bus_->slave_path(), {1, 2});
            FeetechServoHardwareInterface hardware;
            ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);

            auto states = hardware.export_state_interfaces();
            auto commands = hardware.export_command_interfaces();

            ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()),
                      hardware_interface::CallbackReturn::SUCCESS);
            EXPECT_TRUE(bus_->torque_enabled(1));
            EXPECT_TRUE(bus_->torque_enabled(2));
            // 激活时命令值会锁存到实测位置。
            EXPECT_NEAR(commands[0].get_value(), 0.0, 1e-9);

            commands[0].set_value(3.14159265358979323846 / 2.0);
            ASSERT_EQ(hardware.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);
            ASSERT_TRUE(test::wait_for([&] { return bus_->present_position(1) == 818; }));
            EXPECT_EQ(bus_->present_position(1), 818);

            bus_->set_present_speed(1, static_cast<uint16_t>(0x8000U | 100U));
            bus_->set_present_load(1, static_cast<uint16_t>(0x0400U | 500U));
            bus_->set_present_current(1, 100);
            ASSERT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);

            EXPECT_NEAR(states[0].get_value(), expected_tick_radians(818, 300.0), 1e-9);
            EXPECT_NEAR(states[1].get_value(), -100.0 * 300.0 * 3.14159265358979323846 / 180.0 / 1024.0,
                        1e-12);
            EXPECT_NEAR(states[2].get_value(), -0.5, 1e-12);
            EXPECT_NEAR(states[3].get_value(), 7.4, 1e-12);
            EXPECT_NEAR(states[4].get_value(), 30.0, 1e-12);
            EXPECT_NEAR(states[5].get_value(), 0.65, 1e-12);

            ASSERT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()),
                      hardware_interface::CallbackReturn::SUCCESS);
            EXPECT_FALSE(bus_->torque_enabled(1));
            EXPECT_FALSE(bus_->torque_enabled(2));
        }

        TEST_F(HardwareInterfaceTest, ClampsCommandToConfiguredAngleRange) {
            auto info = make_hardware_info(bus_->slave_path(), {1});
            // range_deg = 300：-30 度 -> tick 409，+30 度 -> tick 613。
            info.joints[0].parameters["range_min"] = "-30";
            info.joints[0].parameters["range_max"] = "30";

            FeetechServoHardwareInterface hardware;
            ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
            EXPECT_EQ(bus_->register_value(1, kRegisterMinAngleLimit), 0x01);
            EXPECT_EQ(bus_->register_value(1, kRegisterMinAngleLimit + 1), 0x99);
            EXPECT_EQ(bus_->register_value(1, kRegisterMaxAngleLimit), 0x02);
            EXPECT_EQ(bus_->register_value(1, kRegisterMaxAngleLimit + 1), 0x65);
            auto commands = hardware.export_command_interfaces();
            ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()),
                      hardware_interface::CallbackReturn::SUCCESS);

            commands[0].set_value(3.14159265358979323846 / 2.0);
            ASSERT_EQ(hardware.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);
            ASSERT_TRUE(test::wait_for([&] { return bus_->present_position(1) == 613; }));
            EXPECT_EQ(bus_->present_position(1), 613);

            commands[0].set_value(-3.14159265358979323846 / 2.0);
            ASSERT_EQ(hardware.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);
            ASSERT_TRUE(test::wait_for([&] { return bus_->present_position(1) == 409; }));
            EXPECT_EQ(bus_->present_position(1), 409);
        }

        TEST_F(HardwareInterfaceTest, YamlOverridesUrdfParametersByServoId) {
            const auto yaml_path = std::filesystem::temp_directory_path() / "feetech_joint_config_test.yaml";
            {
                std::ofstream file(yaml_path);
                ASSERT_TRUE(file.is_open());
                file << "joints:\n"
                        "  arm_joint:\n"
                        "    id: 1\n"
                        "    p_coefficient: 33\n"
                        "    range_max: 90\n";
            }

            auto info = make_hardware_info(bus_->slave_path(), {1});
            info.joints[0].parameters["p_coefficient"] = "20";
            info.hardware_parameters["joint_config_file"] = yaml_path.string();

            FeetechServoHardwareInterface hardware;
            ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
            EXPECT_EQ(bus_->register_value(1, kRegisterPCoefficient), 33);
            EXPECT_EQ(bus_->register_value(1, kRegisterMaxAngleLimit), 0x03);
            EXPECT_EQ(bus_->register_value(1, kRegisterMaxAngleLimit + 1), 0x32);

            std::filesystem::remove(yaml_path);
        }

        TEST_F(HardwareInterfaceTest, ConfigFileCanProvideSerialPortAndBaudRate) {
            const auto yaml_path = std::filesystem::temp_directory_path() / "feetech_serial_config_test.yaml";
            {
                std::ofstream file(yaml_path);
                ASSERT_TRUE(file.is_open());
                file << "usb_port: " << bus_->slave_path() << "\n"
                                                              "baud_rate: 115200\n";
            }

            auto info = make_hardware_info("", {1});
            info.hardware_parameters.erase("usb_port");
            info.hardware_parameters["joint_config_file"] = yaml_path.string();

            FeetechServoHardwareInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
            EXPECT_EQ(bus_->register_value(1, kRegisterId), 1);

            std::filesystem::remove(yaml_path);
        }

        TEST_F(HardwareInterfaceTest, ConfiguresTemperatureVoltageAndProtection) {
            auto info = make_hardware_info(bus_->slave_path(), {1});
            auto& params = info.joints[0].parameters;
            params["max_temperature"] = "70";
            params["max_input_voltage"] = "100";
            params["min_input_voltage"] = "50";
            params["unloading_condition"] = "16";
            params["led_alarm_condition"] = "3";
            params["protection_torque"] = "30";
            params["protection_time"] = "50";
            params["overload_torque"] = "70";

            FeetechServoHardwareInterface hardware;
            ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);

            EXPECT_EQ(bus_->register_value(1, kRegisterMaxTemperature), 70);
            EXPECT_EQ(bus_->register_value(1, kRegisterMaxInputVoltage), 100);
            EXPECT_EQ(bus_->register_value(1, kRegisterMinInputVoltage), 50);
            EXPECT_EQ(bus_->register_value(1, kRegisterUnloadingCondition), 16);
            EXPECT_EQ(bus_->register_value(1, kRegisterLedAlarmCondition), 3);
            EXPECT_EQ(bus_->register_value(1, kRegisterProtectionTorque), 30);
            EXPECT_EQ(bus_->register_value(1, kRegisterProtectionTime), 50);
            EXPECT_EQ(bus_->register_value(1, kRegisterOverloadTorque), 70);
            EXPECT_EQ(bus_->register_value(1, kRegisterEepromLock), 1);
        }

        TEST_F(HardwareInterfaceTest, RejectsAngleLimitOutsideMechanicalRange) {
            auto info = make_hardware_info(bus_->slave_path(), {1});
            info.joints[0].parameters["range_min"] = "200";

            FeetechServoHardwareInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
        }

        TEST_F(HardwareInterfaceTest, RejectsInvalidVoltageWindow) {
            auto info = make_hardware_info(bus_->slave_path(), {1});
            info.joints[0].parameters["min_input_voltage"] = "100";
            info.joints[0].parameters["max_input_voltage"] = "90";

            FeetechServoHardwareInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
            EXPECT_EQ(bus_->register_value(1, kRegisterEepromLock), 1);
            EXPECT_EQ(bus_->register_value(1, kRegisterMinInputVoltage), 45);
            EXPECT_EQ(bus_->register_value(1, kRegisterMaxInputVoltage), 90);
        }

        TEST_F(HardwareInterfaceTest, RejectsMissingConfiguredServo) {
            // 模拟总线只提供 id 1 和 2。
            auto info = make_hardware_info(bus_->slave_path(), {3});

            FeetechServoHardwareInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
        }

        TEST_F(HardwareInterfaceTest, RejectsUnsupportedCommandInterface) {
            auto info = make_hardware_info(bus_->slave_path(), {1});
            info.joints[0].command_interfaces[0].name = hardware_interface::HW_IF_VELOCITY;

            FeetechServoHardwareInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
        }

        TEST_F(HardwareInterfaceTest, RejectsParametersMissingOnScsSeries) {
            auto info = make_hardware_info(bus_->slave_path(), {1});
            info.joints[0].parameters["i_coefficient"] = "0";

            FeetechServoHardwareInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
        }

        TEST_F(HardwareInterfaceTest, RejectsDuplicateServoIds) {
            auto info = make_hardware_info(bus_->slave_path(), {1});
            info.joints.push_back(make_joint(1));

            FeetechServoHardwareInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
        }

        TEST(HardwareInterfaceModel, AcceptsSc225Model) {
            FakeScsBus bus(std::vector<uint8_t>{1}, kModelSc225);
            auto info = make_hardware_info(bus.slave_path(), {1});
            info.hardware_parameters["strict_model_check"] = "true";

            FeetechServoHardwareInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
        }

        TEST(HardwareInterfaceModel, StrictModelCheckRejectsUnknownServo) {
            FakeScsBus bus(std::vector<uint8_t>{1}, 0x1234);
            auto info = make_hardware_info(bus.slave_path(), {1});
            info.hardware_parameters["strict_model_check"] = "true";

            FeetechServoHardwareInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
        }

    } // namespace
} // namespace feetech_servo_ros2_control
