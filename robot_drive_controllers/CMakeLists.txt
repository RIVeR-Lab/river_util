cmake_minimum_required(VERSION 2.8.3)
project(robot_drive_controllers)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  roscpp
  geometry_msgs
  river_ros_util
  realtime_tools
  nav_msgs
)


catkin_package(
  CATKIN_DEPENDS controller_interface
  hardware_interface
  roscpp
  geometry_msgs
  river_ros_util
  realtime_tools
  nav_msgs

  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/skid_4wheel_drive_controller.cpp)