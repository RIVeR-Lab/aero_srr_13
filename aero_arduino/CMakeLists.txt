cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_find_ros_package(rosserial_arduino)
include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/rosserial.cmake)


set(FIRMWARE_NAME aero_arduino)

set(${FIRMWARE_NAME}_BOARD mega2560)         # Arduino Target board
set(${FIRMWARE_NAME}_SRCS src/arduino.cpp )
set(${FIRMWARE_NAME}_PORT /dev/ARDUINO)            # Serial upload port
generate_ros_firmware(${FIRMWARE_NAME})
