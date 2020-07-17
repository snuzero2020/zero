cmake_minimum_required(VERSION 2.8.3)
project(clustering)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

# find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  DIRECTORY
  msg
  FILES
  Points.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES clustering
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/clustering.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

