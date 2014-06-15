cmake_minimum_required(VERSION 2.8.3)
project(device_driver_base)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs actionlib actionlib_msgs dynamic_reconfigure river_ros_util message_generation)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES serial_port
  CATKIN_DEPENDS roscpp std_msgs actionlib actionlib_msgs dynamic_reconfigure river_ros_util message_runtime
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(serial_port src/serial_port.cpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS serial_port
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
