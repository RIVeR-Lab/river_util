cmake_minimum_required(VERSION 2.8.3)
project(safety_interface)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  roscpp
  message_generation
  std_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SoftwareStop.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES safety_controllers
  CATKIN_DEPENDS controller_interface roscpp message_runtime std_msgs
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(safety_controllers
  src/safety_topic_controller.cpp
)
add_dependencies(safety_controllers safety_interface_generate_messages_cpp)
target_link_libraries(safety_controllers
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS safety_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/safety_interface
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES safety_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
