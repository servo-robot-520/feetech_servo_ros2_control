#pragma once

#include <optional>
#include <string>
#include <unordered_map>

#include "feetech_servo_ros2_control/result.hpp"

namespace feetech_servo_ros2_control {

    /// @brief 单个关节的参数集合：参数名 -> 字符串值。
    using JointParams = std::unordered_map<std::string, std::string>;
    /// @brief 关节名 -> 参数集合。
    using JointConfigMap = std::unordered_map<std::string, JointParams>;

    /// @brief 解析后的关节配置文件。
    ///
    /// 顶层 `usb_port` 和 `baud_rate` 用于配置串口总线，并优先于 URDF 中同名的
    /// 硬件参数；`joints:` 映射保存关节名 -> 标量参数。
    struct JointConfigFile {
        std::optional<std::string> usb_port;
        std::optional<std::string> baud_rate;
        JointConfigMap joints;
    };

    /// @brief 从 YAML 文件加载总线设置与逐关节参数。
    /// @param file_path YAML 文件路径。
    /// @param config 输出参数；成功时写入解析结果。
    /// @return 文件不存在、格式错误或缺少 `joints:` 映射时返回错误信息。
    Result load_joint_config(const std::string& file_path, JointConfigFile* config);

    /// @brief 合并关节参数，YAML 中的值优先于 URDF `<param>` 中的值。
    /// @param yaml_params 配置文件中的参数。
    /// @param urdf_params URDF 中的参数。
    /// @return 合并后的参数集合。
    JointParams merge_joint_params(const JointParams& yaml_params,
                                   const JointParams& urdf_params);

} // namespace feetech_servo_ros2_control
