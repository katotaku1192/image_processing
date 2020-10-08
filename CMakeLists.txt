cmake_minimum_required(VERSION 3.0.2)
project(image_processing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


catkin_package(CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(depth_red_marker src/depth_red_marker.cpp)
target_link_libraries(depth_red_marker  ${catkin_LIBRARIES})

add_executable(coordinate_red_marker src/coordinate_red_marker.cpp)
target_link_libraries(coordinate_red_marker  ${catkin_LIBRARIES})

