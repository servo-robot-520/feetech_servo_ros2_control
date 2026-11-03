#include "feetech_servo_ros2_control/feetech_servo_hardware_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common_cpp/logging/log_interface/log_macro.hpp"
#include "common_cpp/logging/rclcpp_log_handler.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "feetech_servo_ros2_control/scs_register_map.hpp"

namespace feetech_servo_ros2_control {
    namespace {

        /// @brief 解析整数并校验取值范围。
        /// @param label 参数名称，用于错误信息。
        /// @param text 待解析的字符串。
        /// @param min 允许的最小值。
        /// @param max 允许的最大值。
        /// @param out 解析结果。
        Result parse_integer(const std::string& label, const std::string& text, const int min,
                             const int max, int* const out) {
            try {
                size_t consumed = 0;
                const long value = std::stol(text, &consumed);
                if (consumed != text.size()) {
                    return Result::failure(label + ": expected an integer, got '" + text + "'");
                }
                if (value < min || value > max) {
                    return Result::failure(label + ": " + text + " is outside [" + std::to_string(min) + ", " +
                                           std::to_string(max) + "]");
                }
                *out = static_cast<int>(value);
                return Result::success();
            }
            catch (const std::exception&) {
                return Result::failure(label + ": expected an integer, got '" + text + "'");
            }
        }

        /// @brief 解析浮点数并校验取值范围。
        /// @param label 参数名称，用于错误信息。
        /// @param text 待解析的字符串。
        /// @param min 允许的最小值。
        /// @param max 允许的最大值。
        /// @param out 解析结果。
        Result parse_real(const std::string& label, const std::string& text, const double min,
                          const double max, double* const out) {
            try {
                size_t consumed = 0;
                const double value = std::stod(text, &consumed);
                if (consumed != text.size()) {
                    return Result::failure(label + ": expected a number, got '" + text + "'");
                }
                if (!std::isfinite(value) || value < min || value > max) {
                    return Result::failure(label + ": " + text + " is outside [" + std::to_string(min) + ", " +
                                           std::to_string(max) + "]");
                }
                *out = value;
                return Result::success();
            }
            catch (const std::exception&) {
                return Result::failure(label + ": expected a number, got '" + text + "'");
            }
        }

        /// @brief 解析布尔值（true/false、True/False、1/0）。
        /// @param label 参数名称，用于错误信息。
        /// @param text 待解析的字符串。
        /// @param out 解析结果。
        Result parse_boolean(const std::string& label, const std::string& text, bool* const out) {
            if (text == "true" || text == "True" || text == "1") {
                *out = true;
                return Result::success();
            }
            if (text == "false" || text == "False" || text == "0") {
                *out = false;
                return Result::success();
            }
            return Result::failure(label + ": expected 'true' or 'false', got '" + text + "'");
        }

        /// @brief 将 16 位值格式化为 0xXXXX 字符串。
        std::string hex_word(const uint16_t value) {
            char buffer[8]{};
            std::snprintf(buffer, sizeof(buffer), "0x%04X", static_cast<unsigned>(value));
            return buffer;
        }

        /// @brief 支持的状态接口名称集合。
        const std::set<std::string>& supported_state_interfaces() {
            static const std::set<std::string> kSupported{"position", "velocity", "load", "voltage",
                                                          "temperature"};
            return kSupported;
        }

    } // namespace

    hardware_interface::CallbackReturn FeetechServoHardwareInterface::on_init(
        const hardware_interface::HardwareInfo& info) {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 进程级日志后端，由所有 common_cpp 使用方共享；默认 logger/node 名可被
        // `log.*` 硬件参数覆盖。
        auto logging_parameters = info_.hardware_parameters;
        logging_parameters.try_emplace("log.logger_name", "feetech_servo_ros2_control");
        logging_parameters.try_emplace("log.node_name", "feetech_servo_ros2_control");
        try {
            logging::ros::RclcppLogHandler::init_from_map(logging_parameters);
        }
        catch (const std::exception& error) {
            RCLCPP_ERROR(rclcpp::get_logger("feetech_servo_ros2_control"),
                         "Failed to initialize common_cpp logging: %s", error.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        auto fail = [](const std::string& message) {
            LOG_ERROR("{}", message);
            return hardware_interface::CallbackReturn::ERROR;
        };

        options_ = Options{};
        config_file_ = JointConfigFile{};
        joints_.clear();

        if (const auto it = info_.hardware_parameters.find("joint_config_file");
            it != info_.hardware_parameters.end() && !it->second.empty()) {
            if (const auto result = load_joint_config(it->second, &config_file_); !result) {
                return fail(result.error());
            }
        }

        if (const auto result = parse_options(); result != hardware_interface::CallbackReturn::SUCCESS) {
            return result;
        }
        if (const auto result = build_joints(); result != hardware_interface::CallbackReturn::SUCCESS) {
            return result;
        }

        serial_port_ = std::make_unique<ScsSerialPort>();
        if (const auto result = serial_port_->open(options_.usb_port, options_.baud_rate); !result) {
            return fail(result.error());
        }
        protocol_ = std::make_unique<ScsProtocol>(
            *serial_port_, std::chrono::milliseconds(options_.response_timeout_ms));

        LOG_INFO("hardware: usb_port={} baud_rate={} response_timeout_ms={} range_deg={} default_speed={} "
                 "strict_model_check={}",
                 options_.usb_port, options_.baud_rate, options_.response_timeout_ms, options_.range_deg,
                 options_.default_speed, options_.strict_model_check);

        if (const auto result = verify_servos(); !result) {
            return fail(result.error());
        }

        for (const auto& joint : joints_) {
            if (const auto result = configure_joint(joint); !result) {
                return fail("Joint '" + joint.name + "' (id=" + std::to_string(joint.id) +
                            "): " + result.error());
            }
        }

        LOG_INFO("Configured {} SCS servo(s) on {} at {} baud", joints_.size(), options_.usb_port,
                 options_.baud_rate);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn FeetechServoHardwareInterface::parse_options() {
        auto fail = [](const std::string& message) {
            LOG_ERROR("{}", message);
            return hardware_interface::CallbackReturn::ERROR;
        };

        const auto& parameters = info_.hardware_parameters;

        // 串口设备可以来自 URDF 或关节配置文件；两者都存在时以配置文件为准。
        std::string usb_port;
        if (const auto it = parameters.find("usb_port"); it != parameters.end()) {
            usb_port = it->second;
        }
        if (config_file_.usb_port) {
            usb_port = *config_file_.usb_port;
        }
        if (usb_port.empty()) {
            return fail(
                "Serial device is not configured: set <param name=\"usb_port\">/dev/ttyUSB0</param> "
                "in the URDF or 'usb_port' in joint_config_file");
        }
        options_.usb_port = usb_port;

        int baud_rate = options_.baud_rate; // 默认 1 Mbaud
        if (const auto it = parameters.find("baud_rate"); it != parameters.end()) {
            if (const auto result = parse_integer("baud_rate", it->second, 1, 4000000, &baud_rate);
                !result) {
                return fail(result.error());
            }
        }
        if (config_file_.baud_rate) {
            if (const auto result = parse_integer("joint_config_file: baud_rate",
                                                  *config_file_.baud_rate, 1, 4000000, &baud_rate);
                !result) {
                return fail(result.error());
            }
        }
        options_.baud_rate = baud_rate;

        if (const auto it = parameters.find("response_timeout_ms"); it != parameters.end()) {
            int value = 0;
            if (const auto result = parse_integer("response_timeout_ms", it->second, 1, 1000, &value);
                !result) {
                return fail(result.error());
            }
            options_.response_timeout_ms = value;
        }

        if (const auto it = parameters.find("range_deg"); it != parameters.end()) {
            double value = 0.0;
            if (const auto result = parse_real("range_deg", it->second, 0.1, 360.0, &value); !result) {
                return fail(result.error());
            }
            options_.range_deg = value;
        }

        if (const auto it = parameters.find("default_speed"); it != parameters.end()) {
            int value = 0;
            if (const auto result = parse_integer("default_speed", it->second, 1, 1000, &value); !result) {
                return fail(result.error());
            }
            options_.default_speed = static_cast<uint16_t>(value);
        }

        if (const auto it = parameters.find("strict_model_check"); it != parameters.end()) {
            bool value = false;
            if (const auto result = parse_boolean("strict_model_check", it->second, &value); !result) {
                return fail(result.error());
            }
            options_.strict_model_check = value;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn FeetechServoHardwareInterface::build_joints() {
        auto fail = [](const std::string& message) {
            LOG_ERROR("{}", message);
            return hardware_interface::CallbackReturn::ERROR;
        };

        if (info_.joints.empty()) {
            return fail("At least one joint must be declared for this hardware component");
        }

        std::unordered_map<int, JointParams> yaml_by_id;
        for (const auto& [joint_name, params] : config_file_.joints) {
            const auto id_it = params.find("id");
            if (id_it == params.end()) {
                return fail("joint_config_file: joint '" + joint_name + "' has no 'id' parameter");
            }
            int id = 0;
            if (const auto result = parse_integer("joint_config_file: joint '" + joint_name + "' id",
                                                  id_it->second, 0, kMaxServoId, &id);
                !result) {
                return fail(result.error());
            }
            if (!yaml_by_id.emplace(id, params).second) {
                return fail("joint_config_file: duplicate servo id " + std::to_string(id) +
                            " in joint '" + joint_name + "'");
            }
        }

        joints_.clear();
        joints_.reserve(info_.joints.size());

        std::unordered_set<int> used_ids;
        for (const auto& joint_info : info_.joints) {
            Joint joint;
            joint.name = joint_info.name;

            const auto id_it = joint_info.parameters.find("id");
            if (id_it == joint_info.parameters.end()) {
                return fail("Joint '" + joint.name + "' does not have the required 'id' parameter");
            }
            int id = 0;
            if (const auto result = parse_integer("Joint '" + joint.name + "' id", id_it->second, 0,
                                                  kMaxServoId, &id);
                !result) {
                return fail(result.error());
            }
            if (!used_ids.insert(id).second) {
                return fail("Duplicate servo id " + std::to_string(id) + " (joint '" + joint.name + "')");
            }
            joint.id = static_cast<uint8_t>(id);

            JointParams merged(joint_info.parameters.begin(), joint_info.parameters.end());
            if (const auto yaml_it = yaml_by_id.find(id); yaml_it != yaml_by_id.end()) {
                merged = merge_joint_params(yaml_it->second, merged);
            }

            for (const char* unsupported :
                 {"homing_offset", "offset", "i_coefficient", "acceleration", "protection_current",
                  "max_torque_limit"}) {
                if (merged.find(unsupported) != merged.end()) {
                    return fail("Joint '" + joint.name + "': parameter '" + unsupported +
                                "' is not available on SCS series servos");
                }
            }

            joint.range_deg = options_.range_deg;
            if (const auto it = merged.find("range_deg"); it != merged.end()) {
                if (const auto result = parse_real("Joint '" + joint.name + "' range_deg", it->second, 0.1,
                                                   360.0, &joint.range_deg);
                    !result) {
                    return fail(result.error());
                }
            }

            const double half_range_deg = joint.range_deg / 2.0;
            if (const auto it = merged.find("range_min"); it != merged.end()) {
                double degrees = 0.0;
                if (const auto result = parse_real("Joint '" + joint.name + "' range_min (deg)", it->second,
                                                   -half_range_deg, half_range_deg, &degrees);
                    !result) {
                    return fail(result.error());
                }
                joint.min_tick = degrees_to_tick(degrees, joint.range_deg);
                joint.eeprom_min_tick = joint.min_tick;
            }
            if (const auto it = merged.find("range_max"); it != merged.end()) {
                double degrees = 0.0;
                if (const auto result = parse_real("Joint '" + joint.name + "' range_max (deg)", it->second,
                                                   -half_range_deg, half_range_deg, &degrees);
                    !result) {
                    return fail(result.error());
                }
                joint.max_tick = degrees_to_tick(degrees, joint.range_deg);
                joint.eeprom_max_tick = joint.max_tick;
            }
            if (joint.min_tick >= joint.max_tick) {
                return fail("Joint '" + joint.name + "': range_min (" + std::to_string(joint.min_tick) +
                            " ticks) must be smaller than range_max (" + std::to_string(joint.max_tick) +
                            " ticks)");
            }

            joint.speed = options_.default_speed;
            if (const auto it = merged.find("speed"); it != merged.end()) {
                int value = 0;
                if (const auto result = parse_integer("Joint '" + joint.name + "' speed", it->second, 1, 1000,
                                                      &value);
                    !result) {
                    return fail(result.error());
                }
                joint.speed = static_cast<uint16_t>(value);
            }

            for (const auto& command_interface : joint_info.command_interfaces) {
                if (command_interface.name != hardware_interface::HW_IF_POSITION) {
                    return fail("Joint '" + joint.name + "': unsupported command interface '" +
                                command_interface.name + "' (only 'position' is supported)");
                }
                joint.has_position_command = true;
            }

            for (const auto& state_interface : joint_info.state_interfaces) {
                if (supported_state_interfaces().find(state_interface.name) ==
                    supported_state_interfaces().end()) {
                    return fail("Joint '" + joint.name + "': unsupported state interface '" +
                                state_interface.name + "' (supported: position, velocity, load, voltage, "
                                                       "temperature)");
                }
            }

            joint.params = std::move(merged);
            joints_.push_back(std::move(joint));
        }

        for (const auto& entry : yaml_by_id) {
            if (used_ids.find(entry.first) == used_ids.end()) {
                LOG_WARN("joint_config_file: servo id {} does not match any URDF joint", entry.first);
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    Result FeetechServoHardwareInterface::verify_servos() {
        for (const auto& joint : joints_) {
            if (const auto result = protocol_->ping(joint.id); !result) {
                return Result::failure("Configured servo id " + std::to_string(joint.id) + " (joint '" +
                                       joint.name + "') is not responding, aborting startup: " + result.error());
            }

            std::array<uint8_t, 8> identity{};
            if (const auto result = protocol_->read_registers(joint.id, kRegisterFirmwareMajor, identity); !result) {
                return Result::failure("Configured servo id " + std::to_string(joint.id) + " (joint '" +
                                       joint.name + "'): reading identity failed, aborting startup: " +
                                       result.error());
            }

            const auto model = static_cast<uint16_t>((identity[kRegisterServoMajor] << 8) |
                                                     identity[kRegisterServoMinor]);
            LOG_INFO("servo id={} ({}): firmware={}.{} servo={}.{} model={}", static_cast<unsigned>(joint.id),
                     joint.name, identity[kRegisterFirmwareMajor], identity[kRegisterFirmwareMinor],
                     identity[kRegisterServoMajor], identity[kRegisterServoMinor], model_name(model));

            if (!is_supported_model(model)) {
                LOG_WARN("servo id={} ({}): unsupported model {}", static_cast<unsigned>(joint.id), joint.name,
                         hex_word(model));
                if (options_.strict_model_check) {
                    return Result::failure("Joint '" + joint.name + "': model " + hex_word(model) +
                                           " is not SC215/SC225");
                }
            }

            std::map<std::string, std::string> sorted_params(joint.params.begin(), joint.params.end());
            std::string config_text;
            for (const auto& [key, value] : sorted_params) {
                if (!config_text.empty()) {
                    config_text += " ";
                }
                config_text += key + "=" + value;
            }
            LOG_INFO("servo id={} ({}): config: {}", static_cast<unsigned>(joint.id), joint.name, config_text);
        }

        LOG_INFO("verified {} configured servo(s)", joints_.size());
        return Result::success();
    }

    Result FeetechServoHardwareInterface::configure_joint(const Joint& joint) {
        struct ByteSetting {
            const char* name;
            uint8_t address;
            uint8_t value;
        };
        struct WordSetting {
            const char* name;
            uint8_t address;
            uint16_t value;
        };

        struct ByteParameter {
            const char* name;
            uint8_t address;
            int min;
            int max;
        };
        struct WordParameter {
            const char* name;
            uint8_t address;
            int min;
            int max;
        };

        // 先校验全部配置值，再操作舵机，避免参数错误导致舵机配置到一半。
        std::vector<ByteSetting> byte_settings;
        const std::array<ByteParameter, 13> byte_parameters{{
            {"return_delay_time", kRegisterReturnDelay, 0, 254},
            {"max_temperature", kRegisterMaxTemperature, 0, 100},
            {"max_input_voltage", kRegisterMaxInputVoltage, 0, 254},
            {"min_input_voltage", kRegisterMinInputVoltage, 0, 254},
            {"unloading_condition", kRegisterUnloadingCondition, 0, 254},
            {"led_alarm_condition", kRegisterLedAlarmCondition, 0, 254},
            {"p_coefficient", kRegisterPCoefficient, 0, 254},
            {"d_coefficient", kRegisterDCoefficient, 0, 254},
            {"cw_dead", kRegisterCwDeadZone, 0, 32},
            {"ccw_dead", kRegisterCcwDeadZone, 0, 32},
            {"protection_torque", kRegisterProtectionTorque, 0, 100},
            {"protection_time", kRegisterProtectionTime, 0, 254},
            {"overload_torque", kRegisterOverloadTorque, 0, 100},
        }};
        for (const auto& parameter : byte_parameters) {
            const auto it = joint.params.find(parameter.name);
            if (it == joint.params.end()) {
                continue;
            }
            int value = 0;
            if (const auto result = parse_integer(std::string(parameter.name), it->second, parameter.min,
                                                  parameter.max, &value);
                !result) {
                return result;
            }
            byte_settings.push_back({parameter.name, parameter.address, static_cast<uint8_t>(value)});
        }

        const auto find_byte_setting = [&byte_settings](const char* name) -> const ByteSetting* {
            for (const auto& setting : byte_settings) {
                if (std::string(setting.name) == name) {
                    return &setting;
                }
            }
            return nullptr;
        };
        const ByteSetting* min_voltage = find_byte_setting("min_input_voltage");
        const ByteSetting* max_voltage = find_byte_setting("max_input_voltage");
        if (min_voltage != nullptr && max_voltage != nullptr && min_voltage->value > max_voltage->value) {
            return Result::failure("min_input_voltage must not exceed max_input_voltage");
        }

        std::vector<WordSetting> word_settings;
        const std::array<WordParameter, 1> word_parameters{{
            {"max_torque", kRegisterMaxTorque, 0, 1000},
        }};
        for (const auto& parameter : word_parameters) {
            const auto it = joint.params.find(parameter.name);
            if (it == joint.params.end()) {
                continue;
            }
            int value = 0;
            if (const auto result = parse_integer(std::string(parameter.name), it->second, parameter.min,
                                                  parameter.max, &value);
                !result) {
                return result;
            }
            word_settings.push_back({parameter.name, parameter.address, static_cast<uint16_t>(value)});
        }

        // 角度限制以度为单位（相对中位），写入舵机时换算为 tick。
        if (joint.eeprom_min_tick) {
            word_settings.push_back({"range_min", kRegisterMinAngleLimit, *joint.eeprom_min_tick});
        }
        if (joint.eeprom_max_tick) {
            word_settings.push_back({"range_max", kRegisterMaxAngleLimit, *joint.eeprom_max_tick});
        }

        const auto id = joint.id;
        if (const auto result = protocol_->set_torque_enabled(id, false); !result) {
            return result;
        }
        if (const auto result = protocol_->set_eeprom_locked(id, false); !result) {
            return result;
        }

        for (const auto& setting : byte_settings) {
            std::array<uint8_t, 1> current{};
            if (const auto result = protocol_->read_registers(id, setting.address, current); !result) {
                return Result::failure(std::string("Reading '") + setting.name + "' failed: " +
                                       result.error());
            }
            if (current[0] == setting.value) {
                continue;
            }
            const std::array<uint8_t, 1> desired{setting.value};
            if (const auto result = protocol_->write_registers(id, setting.address, desired); !result) {
                return Result::failure(std::string("Writing '") + setting.name + "' failed: " +
                                       result.error());
            }
            LOG_INFO("servo id={} ({}): {}: current={} configured={} -> update", static_cast<unsigned>(id),
                     joint.name, setting.name, static_cast<unsigned>(current[0]),
                     static_cast<unsigned>(setting.value));
        }

        for (const auto& setting : word_settings) {
            std::array<uint8_t, 2> current{};
            if (const auto result = protocol_->read_registers(id, setting.address, current); !result) {
                return Result::failure(std::string("Reading '") + setting.name + "' failed: " +
                                       result.error());
            }
            if (decode_word(current.data()) == setting.value) {
                continue;
            }
            std::array<uint8_t, 2> desired{};
            encode_word(desired.data(), setting.value);
            if (const auto result = protocol_->write_registers(id, setting.address, desired); !result) {
                return Result::failure(std::string("Writing '") + setting.name + "' failed: " +
                                       result.error());
            }
            const bool angle_limit = std::string(setting.name) == "range_min" ||
                std::string(setting.name) == "range_max";
            LOG_INFO("servo id={} ({}): {}: current={}{} configured={}{} -> update", static_cast<unsigned>(id),
                     joint.name, setting.name, decode_word(current.data()), angle_limit ? " ticks" : "",
                     static_cast<unsigned>(setting.value), angle_limit ? " ticks" : "");
        }

        return protocol_->set_eeprom_locked(id, true);
    }

    std::vector<hardware_interface::StateInterface>
    FeetechServoHardwareInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> interfaces;
        for (size_t i = 0; i < info_.joints.size() && i < joints_.size(); ++i) {
            for (const auto& declared : info_.joints[i].state_interfaces) {
                double* value = nullptr;
                if (declared.name == hardware_interface::HW_IF_POSITION) {
                    value = &joints_[i].state_position;
                }
                else if (declared.name == hardware_interface::HW_IF_VELOCITY) {
                    value = &joints_[i].state_velocity;
                }
                else if (declared.name == "load") {
                    value = &joints_[i].state_load;
                }
                else if (declared.name == "voltage") {
                    value = &joints_[i].state_voltage;
                }
                else if (declared.name == "temperature") {
                    value = &joints_[i].state_temperature;
                }
                if (value != nullptr) {
                    interfaces.emplace_back(info_.joints[i].name, declared.name, value);
                }
            }
        }
        return interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    FeetechServoHardwareInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> interfaces;
        for (size_t i = 0; i < info_.joints.size() && i < joints_.size(); ++i) {
            for (const auto& declared : info_.joints[i].command_interfaces) {
                if (declared.name == hardware_interface::HW_IF_POSITION) {
                    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                                            &joints_[i].command_position);
                }
            }
        }
        return interfaces;
    }

    hardware_interface::return_type FeetechServoHardwareInterface::read(
        const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
        std::array<uint8_t, kFeedbackLength> feedback{};

        for (auto& joint : joints_) {
            if (const auto result = protocol_->read_registers(joint.id, kRegisterPresentPosition, feedback);
                !result) {
                LOG_ERROR("Reading joint '{}' (id={}) failed: {}", joint.name,
                          static_cast<unsigned>(joint.id), result.error());
                return hardware_interface::return_type::ERROR;
            }

            const uint16_t position = decode_word(feedback.data());
            const int speed = decode_sign_magnitude(decode_word(feedback.data() + 2), kSpeedSignBit);
            const int load = decode_sign_magnitude(decode_word(feedback.data() + 4), kLoadSignBit);

            joint.state_position = tick_to_radians(position, joint.range_deg);
            joint.state_velocity = static_cast<double>(speed) * radians_per_tick(joint.range_deg);
            joint.state_load = static_cast<double>(load) / 1000.0;
            joint.state_voltage = static_cast<double>(feedback[6]) * 0.1;
            joint.state_temperature = static_cast<double>(feedback[7]);
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type FeetechServoHardwareInterface::write(
        const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
        std::vector<uint8_t> ids;
        std::vector<uint16_t> positions;
        std::vector<uint16_t> speeds;
        ids.reserve(joints_.size());
        positions.reserve(joints_.size());
        speeds.reserve(joints_.size());

        for (const auto& joint : joints_) {
            if (!joint.has_position_command || !std::isfinite(joint.command_position)) {
                continue;
            }
            ids.push_back(joint.id);
            positions.push_back(radians_to_tick(joint.command_position, joint.range_deg, joint.min_tick,
                                                joint.max_tick));
            speeds.push_back(joint.speed);
        }

        if (ids.empty()) {
            return hardware_interface::return_type::OK;
        }

        if (const auto result = protocol_->sync_write_goal_positions(ids, positions, 0, speeds);
            !result) {
            LOG_ERROR("{}", result.error());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn FeetechServoHardwareInterface::on_activate(
        const rclcpp_lifecycle::State& /* previous_state */) {
        if (read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)) !=
            hardware_interface::return_type::OK) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 使能扭矩前先把命令值锁存到实测位置，避免激活时跳到陈旧目标。
        for (auto& joint : joints_) {
            if (joint.has_position_command) {
                joint.command_position = joint.state_position;
            }
        }

        for (auto& joint : joints_) {
            if (!joint.has_position_command) {
                continue;
            }
            if (const auto result = protocol_->set_torque_enabled(joint.id, true); !result) {
                LOG_ERROR("Enabling torque on joint '{}' (id={}) failed: {}", joint.name,
                          static_cast<unsigned>(joint.id), result.error());
                return hardware_interface::CallbackReturn::ERROR;
            }
            joint.torque_enabled = true;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn FeetechServoHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State& /* previous_state */) {
        bool success = true;
        for (auto& joint : joints_) {
            if (!joint.torque_enabled) {
                continue;
            }
            if (const auto result = protocol_->set_torque_enabled(joint.id, false); !result) {
                LOG_ERROR("Disabling torque on joint '{}' (id={}) failed: {}", joint.name,
                          static_cast<unsigned>(joint.id), result.error());
                success = false;
                continue;
            }
            joint.torque_enabled = false;
        }
        return success ? hardware_interface::CallbackReturn::SUCCESS
                       : hardware_interface::CallbackReturn::ERROR;
    }

} // namespace feetech_servo_ros2_control

PLUGINLIB_EXPORT_CLASS(feetech_servo_ros2_control::FeetechServoHardwareInterface,
                       hardware_interface::SystemInterface)
