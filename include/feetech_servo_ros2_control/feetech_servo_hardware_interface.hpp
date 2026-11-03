#pragma once

#include <limits>
#include <memory>
#include <optional>
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

    /// @brief 飞特 SCS 系列总线舵机的 ros2_control SystemInterface。
    ///
    /// 支持的型号：SC215（型号 0x0523）与 SC225（型号 0x0923）。
    /// 一个实例管理一条串口总线，每个关节对应一个舵机 id。
    class FeetechServoHardwareInterface : public hardware_interface::SystemInterface {
    public:
        /// @brief 解析硬件与关节参数、打开串口、校验已配置的舵机并写入 EEPROM 配置。
        /// @param info ros2_control 从 URDF 解析出的硬件信息。
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        /// @brief 导出 URDF 中声明的状态接口。
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        /// @brief 导出 URDF 中声明的命令接口（仅支持 position）。
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        /// @brief 逐关节顺序读取位置、速度、负载、电压和温度。
        hardware_interface::return_type read(const rclcpp::Time& time,
                                             const rclcpp::Duration& period) override;

        /// @brief 用一次 SYNC WRITE 下发所有被命令关节的目标位置。
        hardware_interface::return_type write(const rclcpp::Time& time,
                                              const rclcpp::Duration& period) override;

        /// @brief 读取当前位置、锁存命令值并使能扭矩。
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state) override;

        /// @brief 关闭所有已使能关节的扭矩。
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state) override;

    private:
        /// @brief 单个关节的运行时状态与配置。
        struct Joint {
            std::string name;
            uint8_t id{};
            JointParams params;
            double range_deg{300.0};
            uint16_t min_tick{0};
            uint16_t max_tick{1023};
            std::optional<uint16_t> eeprom_min_tick;
            std::optional<uint16_t> eeprom_max_tick;
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

        /// @brief 硬件级参数。
        struct Options {
            std::string usb_port;
            int baud_rate{1000000};
            int response_timeout_ms{10};
            double range_deg{300.0};
            uint16_t default_speed{1000};
            bool strict_model_check{false};
        };

        /// @brief 解析 URDF 与配置文件中的硬件级参数。
        hardware_interface::CallbackReturn parse_options();

        /// @brief 按舵机 id 合并 URDF/YAML 参数并校验关节配置。
        hardware_interface::CallbackReturn build_joints();

        /// @brief 把关节的 EEPROM 参数写入舵机，仅在寄存器值不一致时更新。
        Result configure_joint(const Joint& joint);

        /// @brief 校验所有已配置舵机在线，并打印版本与配置信息。
        Result verify_servos();

        Options options_;
        JointConfigFile config_file_;
        std::vector<Joint> joints_;
        std::unique_ptr<ScsSerialPort> serial_port_;
        std::unique_ptr<ScsProtocol> protocol_;
    };

} // namespace feetech_servo_ros2_control
