cmake_minimum_required(VERSION 2.8.3)
project(mbed_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  sensor_msgs
  gecko_msgs
  message_generation
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp serial std_msgs sensor_msgs gecko_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mbed_interface_node
  src/mbed_interface_node.cpp
)

add_library(MbedInterface
  src/MbedInterface.cpp
)

target_link_libraries(mbed_interface_node
  ${catkin_LIBRARIES}
  MbedInterface
)

add_dependencies(mbed_interface_node
  gecko_msgs_generate_messages_cpp
)
