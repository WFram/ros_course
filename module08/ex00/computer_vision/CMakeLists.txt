cmake_minimum_required(VERSION 3.0.2)
project(computer_vision)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  rostime
  std_msgs
  sensor_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES computer_vision
 CATKIN_DEPENDS cv_bridge image_transport roscpp rospy rostime std_msgs
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(publisher_node src/publisher_node.cpp)

add_executable(color_switcher_node src/color_switcher_node.cpp)

add_dependencies(publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(publisher_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(color_switcher_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)