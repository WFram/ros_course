cmake_minimum_required(VERSION 3.0.2)
project(robot_diff_drive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rostime
  std_msgs
  sensor_msgs
  pcl_ros
  actionlib_msgs
  behaviortree_cpp_v3
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES robot_diff_drive
 CATKIN_DEPENDS roscpp rospy rostime std_msgs behaviortree_cpp_v3 actionlib_msgs
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tree src/tree.cpp)

add_executable(battery_status src/battery_status.cpp)

target_link_libraries(tree
  ${catkin_LIBRARIES}
)

target_link_libraries(battery_status
  ${catkin_LIBRARIES}
)