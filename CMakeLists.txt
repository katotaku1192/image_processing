cmake_minimum_required(VERSION 3.0.2)
project(image_processing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
)


catkin_package(CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(coordinate_marker_camera1 src/coordinate_marker_camera1.cpp)
target_link_libraries(coordinate_marker_camera1  ${catkin_LIBRARIES})

add_executable(coordinate_marker_camera2 src/coordinate_marker_camera2.cpp)
target_link_libraries(coordinate_marker_camera2  ${catkin_LIBRARIES})

