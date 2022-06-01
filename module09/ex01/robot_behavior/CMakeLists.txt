cmake_minimum_required(VERSION 3.0.2)
project(robot_behavior)

SET(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  behaviortree_cpp_v3
  roscpp
  rospy
  rostime
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES robot_behavior
 CATKIN_DEPENDS behaviortree_cpp_v3 roscpp rospy rostime std_msgs
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tree src/tree.cpp)

target_link_libraries(tree
  ${catkin_LIBRARIES}
)