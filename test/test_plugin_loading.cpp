#include <gtest/gtest.h>

#include <hardware_interface/system_interface.hpp>

#include <exception>
#include <pluginlib/class_loader.hpp>


TEST(PluginLoading, LoadsFeetechServoHardwareInterface) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");

  pluginlib::UniquePtr<hardware_interface::SystemInterface> instance;
  try {
    instance = loader.createUniqueInstance(
        "feetech_servo_ros2_control/FeetechServoHardwareInterface");
  } catch (const std::exception& error) {
    FAIL() << error.what();
  }
  EXPECT_TRUE(instance != nullptr);
}
