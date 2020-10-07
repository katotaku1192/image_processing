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

add_executable(red_depth src/red_depth.cpp)

target_link_libraries(red_depth  ${catkin_LIBRARIES})

