cmake_minimum_required(VERSION 3.0.2)
project(move_to_goal)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rostime
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES move_to_goal
 CATKIN_DEPENDS roscpp rospy rostime std_msgs
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

message(STATUS "Include: " ${${PROJECT_SOURCE_DIR}/include})

add_executable(move_to_goal src/move_to_goal.cpp)

add_dependencies(move_to_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(move_to_goal
  ${catkin_LIBRARIES}
)