cmake_minimum_required(VERSION 3.0.2)
project(action_turtle_commands)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rostime
  std_msgs
  actionlib_msgs
  actionlib
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES execute_turtle_commands.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES action_turtle_commands
 CATKIN_DEPENDS actionlib_msgs roscpp rospy rostime std_msgs
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(action_server src/action_server.cpp)
add_executable(action_client src/action_client.cpp)

add_dependencies(action_server ${action_turtle_commands_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(action_client ${action_turtle_commands_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(action_server
  ${catkin_LIBRARIES}
)

target_link_libraries(action_client
  ${catkin_LIBRARIES}
)
