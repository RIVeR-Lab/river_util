cmake_minimum_required(VERSION 2.8.3)
project(stereo_image_sync)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  nodelet
  river_ros_util
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  nodelet
  message_runtime
  river_ros_util
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
