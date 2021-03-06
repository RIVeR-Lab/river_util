cmake_minimum_required(VERSION 2.8.3)
project(robot_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)


find_package(PkgConfig)
pkg_check_modules(Curses ncurses)

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS roscpp geometry_msgs message_runtime
  DEPENDS Curses
)


include_directories(
  ${Curses_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(keyboard_twist_teleop src/keyboard_twist.cpp)
target_link_libraries(keyboard_twist_teleop ${catkin_LIBRARIES} ${Curses_LIBRARIES})

add_executable(joystick_twist_teleop src/joystick_twist.cpp)
target_link_libraries(joystick_twist_teleop ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS keyboard_twist_teleop joystick_twist_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
