# feetech_servo_ros2_control

[English](README.md) | [简体中文](README.zh-CN.md)

`ros2_control` hardware interface for Feetech **SCS** series bus servos, targeting
the **SC215** and **SC225** (potentiometer, 10-bit encoder, TTL half-duplex bus).

The driver speaks the SCSCL protocol directly over a POSIX serial port:

- 8N1 raw serial, arbitrary baud rates (1 Mbaud by default),
- 0..1023 tick positions with the factory midpoint at 511,
- high-byte-first 16-bit register encoding,
- READ / WRITE / SYNC WRITE, torque and EEPROM lock control,
- sequential feedback reads (the SCS protocol manual does not define SYNC READ).

## Build

```bash
colcon build --packages-select feetech_servo_ros2_control
colcon test  --packages-select feetech_servo_ros2_control
```

Dependencies: ROS 2 Humble (`rclcpp`, `rclcpp_lifecycle`, `hardware_interface`,
`pluginlib`), `common_cpp` and `yaml-cpp`. `common_cpp` requires a C++23-capable
standard library (GCC 13+), so build the workspace with GCC 13. The serial device
must be accessible, typically:

```bash
sudo usermod -aG dialout "$USER"
```

## ros2_control configuration

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

Only state interfaces declared in the URDF are exported. Supported state
interfaces: `position`, `velocity`, `load`, `voltage`, `temperature`.
Only the `position` command interface is supported.

### Hardware parameters

| Parameter | Default | Description |
|---|---|---|
| `usb_port` | — | Serial device. Required either here or as `usb_port` in the joint config file. |
| `baud_rate` | `1000000` | Bus baud rate. The joint config file overrides this value. |
| `joint_config_file` | — | Optional YAML file with bus settings and per-joint calibration. |
| `response_timeout_ms` | `10` | Timeout for one servo response. |
| `range_deg` | `300` | Mechanical travel of the servo across 0..1023 ticks. Per-joint `range_deg` overrides it. |
| `default_speed` | `1000` | Command speed in ticks/s when a joint does not set `speed`. |
| `strict_model_check` | `false` | Reject servos whose model is not SC215/SC225. |

The serial device and baud rate may also be provided by the joint config file
(top-level `usb_port` / `baud_rate`); the config file takes precedence over the
URDF parameters.

### Startup verification

`on_init` only contacts the servo ids configured in the URDF / joint config file;
there is no full-bus scan. For each configured joint it:

1. pings the id and aborts initialization with an error log if the servo does not
   answer;
2. reads and prints the firmware major/minor version (registers 0/1), the servo
   major/minor version (registers 3/4) and the model derived from them;
3. prints every configured parameter for the joint;
4. compares each register-backed parameter with the servo value and logs
   `current=... configured=... -> update` before writing a different value.

The configured baud rate is the one used on the bus, so no baud rate is printed.

### Logging

Diagnostics use the `common_cpp` logging backend (spdlog console/file sinks with
the ROS bridge). It is a process-global backend, initialized from the hardware
parameters on `on_init`:

| Parameter | Default | Description |
|---|---|---|
| `log.log_level` | `info` | `trace`, `debug`, `info`, `warn`, `error`, `critical`, `off`. |
| `log.log_path` | `./log` | Directory for `log_<name>_<timestamp>.log`. |
| `log.async` | `true` | Asynchronous spdlog queue. |
| `log.logger_name` | `feetech_servo_ros2_control` | Logger name, also used by the ROS adapter. |
| `log.node_name` | `feetech_servo_ros2_control` | Prefix shown in log lines. |
| `log.filter.black` | empty | Comma-separated filter rules (`start-`, `end-`, `inc-`, `equal-`). |

### Joint parameters

| Parameter | Register | Range | Notes |
|---|---|---|---|
| `id` | — | 0..253 | **Required**. Must be unique on the bus. |
| `range_min` | 9 (2 B) | −range_deg/2..+range_deg/2 | Lower command/EEPROM limit, degrees relative to center. |
| `range_max` | 11 (2 B) | −range_deg/2..+range_deg/2 | Upper command/EEPROM limit, degrees relative to center. |
| `range_deg` | — | 0.1..360 | Mechanical travel used for the tick/radian conversion. |
| `speed` | — | 1..1000 | Command speed in ticks/s. |
| `return_delay_time` | 7 | 0..254 | 2 µs units. |
| `p_coefficient` | 21 | 0..254 | Position loop P gain. |
| `d_coefficient` | 22 | 0..254 | Position loop D gain. |
| `cw_dead` | 26 | 0..32 | Clockwise dead zone, ticks. |
| `ccw_dead` | 27 | 0..32 | Counter-clockwise dead zone, ticks. |
| `max_temperature` | 13 | 0..100 | Maximum operating temperature, °C. |
| `max_input_voltage` | 14 | 0..254 | Maximum input voltage, 0.1 V units (90 = 9.0 V). |
| `min_input_voltage` | 15 | 0..254 | Minimum input voltage, 0.1 V units (45 = 4.5 V). |
| `max_torque` | 16 (2 B) | 0..1000 | Maximum torque, 1000 = 100 % stall torque. |
| `unloading_condition` | 19 | 0..254 | Unloading-condition bit field (which faults release torque). |
| `led_alarm_condition` | 20 | 0..254 | LED-alarm condition bit field. |
| `protection_torque` | 37 | 0..100 | Torque output after overload protection, percent. |
| `protection_time` | 38 | 0..254 | Overload hold time, 40 ms units. |
| `overload_torque` | 39 | 0..100 | Overload threshold, percent of maximum torque. |

`min_input_voltage` must not exceed `max_input_voltage`.

EEPROM parameters are written only when the servo value differs, with torque
disabled and the EEPROM unlocked. Parameters that exist on STS servos but not on
SCS potentiometer servos are rejected with an error instead of being silently
ignored: `homing_offset`, `offset`, `i_coefficient`, `acceleration`,
`protection_current`, `max_torque_limit`.

## Joint config file

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

Entries are matched to URDF joints by **servo id**; the YAML joint name is only
a label. YAML values override URDF `<param>` values for the same id.

## Angle convention

Positions and velocities use the configured mechanical range:

```
radians = (ticks - 511) * range_deg * pi / 180 / 1024
```

`range_deg` defaults to 300 (SC215). SC225 has 180° and 300° variants, so set
`range_deg` for that joint to match the actual servo. `range_min`/`range_max`
use the same degree convention (0° is the center, tick 511) and are valid within
±`range_deg`/2. They are converted to ticks for command clamping and for the
servo EEPROM angle-limit registers. Commands are rounded to ticks and clamped to
`range_min..range_max` before being sent.

## State interfaces

| Interface | Unit | Source |
|---|---|---|
| `position` | rad | present position, tick 511 = 0 rad |
| `velocity` | rad/s | present speed (sign bit 15), ticks/s |
| `load` | -1..1 | present load (sign bit 10), 1000 = 100 %; read every cycle |
| `voltage` | V | present voltage, 0.1 V units |
| `temperature` | °C | present temperature |

## Lifecycle

- `on_init`: opens the bus, reads and logs the model, writes explicit EEPROM
  settings (torque off, EEPROM unlocked, then locked again).
- `on_activate`: reads current positions, latches commands to them, then enables
  torque on joints with a position command interface.
- `read`: one sequential 8-byte read per joint at `0x38`.
- `write`: one SYNC WRITE at `0x2A` for all commanded joints.
- `on_deactivate`: disables torque.

A servo that does not answer within `response_timeout_ms` makes `read`/`write`
return `ERROR`.

## Limitations

- No SYNC READ: feedback is read sequentially, one transaction per joint per
  cycle.
- SCS potentiometer servos have no EEPROM homing offset, no I term and no
  acceleration register, so those STS-oriented parameters are unavailable.
- The servo must answer WRITE commands (`Response Level` register 8 = 1, the
  factory default).
- Tested against a protocol-faithful pty simulator and a physical SC225;
  SC215 hardware validation is still required.
