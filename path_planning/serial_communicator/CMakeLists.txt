cmake_minimum_required(VERSION 2.8.3)
project(serial_communicator)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  core_msgs
)

find_package(async_comm REQUIRED)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp std_msgs core_msgs
  DEPENDS async_comm
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${async_comm_INCLUDE_DIRS}
)

add_executable(input_signal scripts/input_signal.cpp)
target_link_libraries(input_signal ${catkin_LIBRARIES})
add_dependencies(input_signal core_msgs_generate_messages_cpp)
#add_dependencies(serial_communicator_manual serial_communicator_generate_messages_cpp)

target_link_libraries(input_signal ${async_comm_LIBRARIES})
