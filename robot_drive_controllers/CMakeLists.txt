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
target_link_libraries(skid_4wheel_drive_controller ${catkin_LIBRARIES})

add_executable(skid_4wheel_drive_controller_node src/skid_4wheel_drive_controller_node.cpp)
target_link_libraries(skid_4wheel_drive_controller_node ${catkin_LIBRARIES} skid_4wheel_drive_controller)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS skid_4wheel_drive_controller skid_4wheel_drive_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/robot_drive_controllers
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
