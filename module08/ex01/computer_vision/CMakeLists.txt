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
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/ImgProcessing.cfg
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES computer_vision
 CATKIN_DEPENDS cv_bridge image_transport roscpp rospy rostime std_msgs sensor_msgs dynamic_reconfigure
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(publisher_node src/publisher_node.cpp)

add_executable(server src/server.cpp)

add_dependencies(server ${PROJECT_NAME}_gencfg)

target_link_libraries(publisher_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(server
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)