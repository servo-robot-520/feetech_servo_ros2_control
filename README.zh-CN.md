# feetech_servo_ros2_control

[English](README.md) | [简体中文](README.zh-CN.md)

`ros2_control` 硬件接口，面向飞特 **SCS** 系列总线舵机，支持 **SC215** 与 **SC225**
（电位器、10 位编码器、TTL 半双工总线）。

驱动通过 POSIX 串口直接实现 SCSCL 协议：

- 8N1 原始串口，支持任意波特率（默认 1 Mbaud）；
- 0..1023 tick 位置，出厂中位为 511；
- 16 位寄存器高字节在前；
- READ / WRITE / SYNC WRITE、扭矩与 EEPROM 锁控制；
- 逐关节顺序读取反馈（SCS 协议手册未定义 SYNC READ）。

## 构建

```bash
colcon build --packages-select feetech_servo_ros2_control
colcon test  --packages-select feetech_servo_ros2_control
```

依赖：ROS 2 Humble（`rclcpp`、`rclcpp_lifecycle`、`hardware_interface`、
`pluginlib`）、`common_cpp` 与 `yaml-cpp`。`common_cpp` 需要支持 C++23 的标准库
（GCC 13+），因此请使用 GCC 13 构建工作区。串口设备需要可访问，通常：

```bash
sudo usermod -aG dialout "$USER"
```

## ros2_control 配置

```xml
<ros2_control name="feetech_scs" type="system">
  <hardware>
    <plugin>feetech_servo_ros2_control/FeetechServoHardwareInterface</plugin>
    <param name="usb_port">/dev/ttyUSB0</param>
    <param name="baud_rate">1000000</param>
    <param name="joint_config_file">$(find my_robot_bringup)/config/scs_joints.yaml</param>
  </hardware>

  <joint name="shoulder_pan_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="load"/>
    <state_interface name="voltage"/>
    <state_interface name="temperature"/>
    <param name="id">1</param>
    <param name="range_min">-90</param>
    <param name="range_max">90</param>
    <param name="p_coefficient">16</param>
    <param name="d_coefficient">16</param>
  </joint>
</ros2_control>
```

只有 URDF 中声明的状态接口才会导出。支持的状态接口：`position`、`velocity`、
`load`、`voltage`、`temperature`。命令接口仅支持 `position`。

### 硬件参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `usb_port` | — | 串口设备。可在此处或关节配置文件的 `usb_port` 中提供。 |
| `baud_rate` | `1000000` | 总线波特率。关节配置文件会覆盖该值。 |
| `joint_config_file` | — | 可选 YAML 文件，包含总线设置与逐关节标定参数。 |
| `response_timeout_ms` | `10` | 单个舵机响应超时。 |
| `range_deg` | `300` | 舵机在 0..1023 tick 范围内的机械行程。关节级 `range_deg` 可覆盖。 |
| `default_speed` | `1000` | 关节未设置 `speed` 时的命令速度，单位 tick/s。 |
| `strict_model_check` | `false` | 拒绝型号不是 SC215/SC225 的舵机。 |

串口设备与波特率也可以通过关节配置文件提供（顶层 `usb_port` / `baud_rate`）；
配置文件优先于 URDF 参数。

### 启动校验

`on_init` 只访问 URDF / 关节配置文件中定义的舵机 id，不再扫描整条总线。对每个已配置
关节：

1. ping 该 id；舵机无应答时输出错误日志并中止启动；
2. 读取并打印固件主/次版本（寄存器 0/1）、舵机主/次版本（寄存器 3/4）及其推导出的型号；
3. 打印该关节配置的全部参数；
4. 逐项读取寄存器当前值，不一致时打印 `current=... configured=... -> update` 并写入新值。

总线使用配置文件中设置的波特率，因此不再打印波特率。

### 日志

诊断信息使用 `common_cpp` 日志后端（spdlog 控制台/文件 sink 与 ROS 桥接）。它是进程级
后端，在 `on_init` 时从硬件参数初始化：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `log.log_level` | `info` | `trace`、`debug`、`info`、`warn`、`error`、`critical`、`off`。 |
| `log.log_path` | `./log` | 日志目录，文件名为 `log_<name>_<timestamp>.log`。 |
| `log.async` | `true` | 使用 spdlog 异步队列。 |
| `log.logger_name` | `feetech_servo_ros2_control` | 日志器名称，同时用于 ROS 适配器。 |
| `log.node_name` | `feetech_servo_ros2_control` | 日志行中显示的前缀。 |
| `log.filter.black` | 空 | 逗号分隔的过滤规则（`start-`、`end-`、`inc-`、`equal-`）。 |

### 关节参数

| 参数 | 寄存器 | 范围 | 说明 |
|---|---|---|---|
| `id` | — | 0..253 | **必填**。总线上必须唯一。 |
| `range_min` | 9（2 字节） | −range_deg/2..+range_deg/2 | 命令/EEPROM 下限，单位度，相对中位。 |
| `range_max` | 11（2 字节） | −range_deg/2..+range_deg/2 | 命令/EEPROM 上限，单位度，相对中位。 |
| `range_deg` | — | 0.1..360 | 用于 tick/弧度换算的机械行程。 |
| `speed` | — | 1..1000 | 命令速度，单位 tick/s。 |
| `return_delay_time` | 7 | 0..254 | 单位 2 µs。 |
| `p_coefficient` | 21 | 0..254 | 位置环 P 增益。 |
| `d_coefficient` | 22 | 0..254 | 位置环 D 增益。 |
| `cw_dead` | 26 | 0..32 | 顺时针死区，单位 tick。 |
| `ccw_dead` | 27 | 0..32 | 逆时针死区，单位 tick。 |
| `max_temperature` | 13 | 0..100 | 最高工作温度，单位 °C。 |
| `max_input_voltage` | 14 | 0..254 | 最高输入电压，单位 0.1 V（90 = 9.0 V）。 |
| `min_input_voltage` | 15 | 0..254 | 最低输入电压，单位 0.1 V（45 = 4.5 V）。 |
| `max_torque` | 16（2 字节） | 0..1000 | 最大扭矩，1000 = 100% 堵转扭矩。 |
| `unloading_condition` | 19 | 0..254 | 卸载条件位域（满足哪些故障时释放扭矩）。 |
| `led_alarm_condition` | 20 | 0..254 | LED 报警条件位域。 |
| `protection_torque` | 37 | 0..100 | 过载保护后的输出扭矩，单位 %。 |
| `protection_time` | 38 | 0..254 | 过载保持时间，单位 40 ms。 |
| `overload_torque` | 39 | 0..100 | 过载阈值，单位为最大扭矩的百分比。 |

`min_input_voltage` 不得大于 `max_input_voltage`。

EEPROM 参数仅在舵机当前值不同时写入，写入前关闭扭矩并解锁 EEPROM。STS 舵机存在但 SCS
电位器舵机不存在的参数会直接报错，而不是静默忽略：`homing_offset`、`offset`、
`i_coefficient`、`acceleration`、`protection_current`、`max_torque_limit`。

## 关节配置文件

```yaml
usb_port: /dev/ttyUSB0
baud_rate: 1000000

joints:
  shoulder_pan_joint:
    id: 1
    p_coefficient: 16
    d_coefficient: 16
    range_min: -60
    range_max: 60
    range_deg: 300
    speed: 800
    max_temperature: 70
    max_input_voltage: 100
    min_input_voltage: 50
    unloading_condition: 32
    led_alarm_condition: 37
    protection_torque: 20
    protection_time: 100
    overload_torque: 80
  elbow_joint:
    id: 2
    range_min: -80
    range_max: 80
```

条目按**舵机 id** 匹配 URDF 关节；YAML 中的关节名只是标签。相同 id 下 YAML 值覆盖
URDF `<param>` 值。

## 角度约定

位置与速度使用配置的机械行程：

```
radians = (ticks - 511) * range_deg * pi / 180 / 1024
```

`range_deg` 默认 300（SC215）。SC225 有 180° 与 300° 两种规格，请按实际舵机为该关节
设置 `range_deg`。`range_min`/`range_max` 使用同一套角度约定（0° 为中位，即 tick 511），
有效范围为 ±`range_deg`/2；写命令钳位和舵机 EEPROM 角度限制寄存器时会换算成 tick。
命令会四舍五入到 tick，并钳位到 `range_min..range_max` 后发送。

## 状态接口

| 接口 | 单位 | 来源 |
|---|---|---|
| `position` | rad | 当前位置，tick 511 = 0 rad |
| `velocity` | rad/s | 当前速度（符号位 15），单位 tick/s |
| `load` | -1..1 | 当前负载（符号位 10），1000 = 100%；每周期读取 |
| `voltage` | V | 当前电压，单位 0.1 V |
| `temperature` | °C | 当前温度 |

## 生命周期

- `on_init`：打开总线，读取并打印型号，写入显式配置的 EEPROM 参数（关闭扭矩、解锁
  EEPROM，写完后再上锁）。
- `on_activate`：读取当前位置，把命令值锁存到当前位置，然后为带 position 命令接口的
  关节使能扭矩。
- `read`：每个关节在 `0x38` 起始处顺序读取 8 字节。
- `write`：对全部被命令的关节在 `0x2A` 起始处发送一次 SYNC WRITE。
- `on_deactivate`：关闭扭矩。

舵机在 `response_timeout_ms` 内未应答时，`read`/`write` 返回 `ERROR`。

## 限制

- 不支持 SYNC READ：反馈逐关节顺序读取，每周期每个关节一次事务。
- SCS 电位器舵机没有 EEPROM 回中偏移、I 项与加速度寄存器，因此这些 STS 参数不可用。
- 舵机必须对 WRITE 指令应答（`Response Level` 寄存器 8 = 1，出厂默认）。
- 已通过协议级 pty 模拟器与实机 SC225 验证；SC215 仍需实机验证。
