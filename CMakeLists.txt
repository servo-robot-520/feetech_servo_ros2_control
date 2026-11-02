cmake_minimum_required(VERSION 3.16)
project(feetech_servo_ros2_control)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)

# Core library: POSIX serial transport, SCS protocol codec, YAML joint configuration.
add_library(feetech_servo_driver SHARED
  src/scs_serial_port.cpp
  src/scs_protocol.cpp
  src/joint_config.cpp)
target_include_directories(
  feetech_servo_driver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                              $<INSTALL_INTERFACE:include>)
target_link_libraries(feetech_servo_driver PUBLIC yaml-cpp)

# ros2_control plugin.
add_library(feetech_servo_ros2_control SHARED src/feetech_servo_hardware_interface.cpp)
target_include_directories(
  feetech_servo_ros2_control PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                    $<INSTALL_INTERFACE:include>)
target_link_libraries(
  feetech_servo_ros2_control PUBLIC feetech_servo_driver
                                   hardware_interface::hardware_interface
                                   pluginlib::pluginlib rclcpp::rclcpp
                                   rclcpp_lifecycle::rclcpp_lifecycle)

pluginlib_export_plugin_description_file(
  hardware_interface feetech_servo_ros2_control_plugin_description.xml)

install(
  TARGETS feetech_servo_driver feetech_servo_ros2_control
  EXPORT feetech_servo_ros2_controlTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(
  FILES feetech_servo_ros2_control_plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(feetech_servo_ros2_controlTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib
                          yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_scs_protocol test/test_scs_protocol.cpp)
  target_link_libraries(test_scs_protocol feetech_servo_driver)

  ament_add_gtest(test_scs_serial_port test/test_scs_serial_port.cpp)
  target_include_directories(test_scs_serial_port PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(test_scs_serial_port feetech_servo_driver)

  ament_add_gtest(test_hardware_interface test/test_hardware_interface.cpp)
  target_include_directories(test_hardware_interface PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(test_hardware_interface feetech_servo_ros2_control)

  ament_add_gtest(test_plugin_loading test/test_plugin_loading.cpp)
  target_link_libraries(test_plugin_loading feetech_servo_ros2_control
                        pluginlib::pluginlib)
endif()

ament_package()
