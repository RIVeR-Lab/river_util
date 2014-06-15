cmake_minimum_required(VERSION 2.8.3)
project(river_ros_util)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/river_ros_util
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
