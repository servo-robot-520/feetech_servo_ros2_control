cmake_minimum_required(VERSION 3.16)
project(feetech_servo_ros2_control)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
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

install(
  TARGETS feetech_servo_driver
  EXPORT feetech_servo_ros2_controlTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(feetech_servo_ros2_controlTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp)

ament_package()
