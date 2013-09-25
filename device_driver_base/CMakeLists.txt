cmake_minimum_required(VERSION 2.8.3)
project(device_driver_base)

find_package(catkin REQUIRED roscpp std_msgs actionlib actionlib_msgs dynamic_reconfigure river_ros_util)

add_action_files(DIRECTORY action FILES SetJointPosition.action SetMotorPosition.action)
add_message_files(DIRECTORY msg FILES MotorFeedback.msg SensorLevels.msg SetJointVelocity.msg SetMotorPower.msg SetMotorVelocity.msg)
generate_messages(DEPENDENCIES actionlib_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES serial_port device_driver
  CATKIN_DEPENDS roscpp std_msgs actionlib actionlib_msgs dynamic_reconfigure river_ros_util
  DEPENDS sensors
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(serial_port src/serial_port.cpp)
add_library(device_driver src/device_driver.cpp)

