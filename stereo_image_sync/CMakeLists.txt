cmake_minimum_required(VERSION 2.8.3)
project(stereo_image_sync)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  nodelet
  river_ros_util
  diagnostic_updater
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stereo_image_sync
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  nodelet
  message_runtime
  river_ros_util
  diagnostic_updater
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(stereo_image_sync src/stereo_image_sync.cpp)
target_link_libraries(stereo_image_sync ${catkin_LIBRARIES})

add_library(stereo_image_sync_nodelet src/image_sync_nodelet.cpp)
target_link_libraries(stereo_image_sync_nodelet stereo_image_sync ${catkin_LIBRARIES})

add_executable(stereo_image_sync_node src/image_sync_node.cpp)
target_link_libraries(stereo_image_sync_node stereo_image_sync ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS stereo_image_sync stereo_image_sync_nodelet stereo_image_sync_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/stereo_image_sync
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
