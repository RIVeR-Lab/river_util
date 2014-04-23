cmake_minimum_required(VERSION 2.8.3)
project(robot_drive_controllers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  river_ros_util
  nav_msgs
)


catkin_package(
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  river_ros_util
  nav_msgs

  INCLUDE_DIRS include
  LIBRARIES skid_4wheel_drive_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(skid_4wheel_drive_controller
    src/skid_4wheel_drive_controller.cpp)

add_executable(skid_4wheel_drive_controller_node src/skid_4wheel_drive_controller_node.cpp)
target_link_libraries(skid_4wheel_drive_controller_node ${catkin_LIBRARIES} skid_4wheel_drive_controller)