#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "feetech_servo_ros2_control/joint_config.hpp"
#include "feetech_servo_ros2_control/scs_protocol.hpp"
#include "feetech_servo_ros2_control/scs_serial_port.hpp"

namespace feetech_servo_ros2_control {

    /// ros2_control SystemInterface for Feetech SCS series bus servos.
    ///
    /// Supported models: SC215 (model 0x2305) and SC225 (model 0x2309).
    /// One instance owns one serial bus; each joint maps to one servo id.
    class FeetechServoHardwareInterface : public hardware_interface::SystemInterface {
    public:
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time& time,
                                             const rclcpp::Duration& period) override;
        hardware_interface::return_type write(const rclcpp::Time& time,
                                              const rclcpp::Duration& period) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state) override;

    private:
        struct Joint {
            std::string name;
            uint8_t id{};
            JointParams params;
            double range_deg{300.0};
            uint16_t min_tick{0};
            uint16_t max_tick{1023};
            uint16_t speed{1000};
            bool has_position_command{false};
            bool torque_enabled{false};
            double state_position{0.0};
            double state_velocity{0.0};
            double state_load{0.0};
            double state_voltage{0.0};
            double state_temperature{0.0};
            double command_position{std::numeric_limits<double>::quiet_NaN()};
        };

        struct Options {
            std::string usb_port;
            int baud_rate{1000000};
            int response_timeout_ms{10};
            double range_deg{300.0};
            uint16_t default_speed{1000};
            bool strict_model_check{false};
        };

        hardware_interface::CallbackReturn parse_options();
        hardware_interface::CallbackReturn build_joints();
        Result configure_joint(const Joint& joint);
        Result check_models();

        Options options_;
        JointConfigFile config_file_;
        std::vector<Joint> joints_;
        std::unique_ptr<ScsSerialPort> serial_port_;
        std::unique_ptr<ScsProtocol> protocol_;
    };

} // namespace feetech_servo_ros2_control
