cmake_minimum_required(VERSION 3.0.2)
project(ros_mavlink)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ros_mavlink
 CATKIN_DEPENDS roscpp std_msgs serial
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_mavlink_node 
src/ros_mavlink_node.cpp
include/interface/serial_port.cpp
include/ros_mavlink/ros_wrapper_mavlink.cpp)

target_link_libraries(ros_mavlink_node
  ${catkin_LIBRARIES}
)