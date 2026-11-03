#include "feetech_servo_ros2_control/joint_config.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace feetech_servo_ros2_control {

    Result load_joint_config(const std::string& file_path, JointConfigFile* const config) {
        if (config == nullptr) {
            return Result::failure("load_joint_config: output is null");
        }
        *config = JointConfigFile{};

        if (std::error_code error; !std::filesystem::exists(file_path, error)) {
            return Result::failure("joint_config_file '" + file_path + "' does not exist");
        }

        std::ifstream file(file_path);
        if (!file.is_open()) {
            return Result::failure("Failed to open joint_config_file '" + file_path + "'");
        }

        try {
            const YAML::Node root = YAML::Load(file);

            // 顶层总线参数可覆盖 URDF 中的同名硬件参数。
            if (const YAML::Node usb_port = root["usb_port"]; usb_port) {
                if (!usb_port.IsScalar() || usb_port.Scalar().empty()) {
                    return Result::failure("joint_config_file '" + file_path +
                                           "': 'usb_port' must be a non-empty string");
                }
                config->usb_port = usb_port.Scalar();
            }

            if (const YAML::Node baud_rate = root["baud_rate"]; baud_rate) {
                if (!baud_rate.IsScalar()) {
                    return Result::failure("joint_config_file '" + file_path +
                                           "': 'baud_rate' must be a scalar");
                }
                config->baud_rate = baud_rate.Scalar();
            }

            // 关节名仅作标签，驱动按 id 将配置匹配到 URDF 关节。
            const YAML::Node joints = root["joints"];
            if (joints && !joints.IsMap()) {
                return Result::failure("joint_config_file '" + file_path +
                                       "': top-level 'joints' must be a map");
            }

            for (auto it = joints.begin(); it != joints.end(); ++it) {
                const auto joint_name = it->first.as<std::string>();
                const YAML::Node joint_node = it->second;
                if (!joint_node.IsMap()) {
                    return Result::failure("joint_config_file '" + file_path + "': joint '" + joint_name +
                                           "' is not a map");
                }

                JointParams params;
                for (auto param = joint_node.begin(); param != joint_node.end(); ++param) {
                    const auto key = param->first.as<std::string>();
                    if (!param->second.IsScalar()) {
                        return Result::failure("joint_config_file '" + file_path + "': parameter '" + key +
                                               "' of joint '" + joint_name + "' is not a scalar");
                    }
                    params[key] = param->second.Scalar();
                }
                config->joints.insert_or_assign(joint_name, std::move(params));
            }
        }
        catch (const YAML::Exception& error) {
            return Result::failure("YAML parsing error in '" + file_path + "': " + error.what());
        }
        catch (const std::exception& error) {
            return Result::failure("Failed to load joint_config_file '" + file_path + "': " + error.what());
        }

        return Result::success();
    }

    JointParams merge_joint_params(const JointParams& yaml_params, const JointParams& urdf_params) {
        JointParams merged = urdf_params;
        for (const auto& [key, value] : yaml_params) {
            merged[key] = value;
        }
        return merged;
    }

} // namespace feetech_servo_ros2_control
