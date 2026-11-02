#pragma once

#include <optional>
#include <string>
#include <unordered_map>

#include "feetech_servo_ros2_control/result.hpp"

namespace feetech_servo_ros2_control {

    using JointParams = std::unordered_map<std::string, std::string>;
    using JointConfigMap = std::unordered_map<std::string, JointParams>;

    /// Parsed joint configuration file.
    ///
    /// Top-level keys `usb_port` and `baud_rate` configure the serial bus and take
    /// precedence over the matching URDF hardware parameters. The `joints:` map
    /// contains joint name -> scalar parameters.
    struct JointConfigFile {
        std::optional<std::string> usb_port;
        std::optional<std::string> baud_rate;
        JointConfigMap joints;
    };

    Result load_joint_config(const std::string& file_path, JointConfigFile* config);

    /// YAML values take precedence over URDF `<param>` values.
    JointParams merge_joint_params(const JointParams& yaml_params,
                                   const JointParams& urdf_params);

} // namespace feetech_servo_ros2_control
