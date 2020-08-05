cmake_minimum_required(VERSION 3.0.2)
project(pid_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  core_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  slam
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs core_msgs slam
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(PID_tracker_node src/pid_tracker.cpp)
add_executable(pub_vel src/publisher.cpp)
add_executable(pid_control_node src/pid_control.cpp)

add_dependencies(pub_vel core_msgs_generate_messages_cpp)
add_dependencies(PID_tracker_node core_msgs_generate_messages_cpp)
add_dependencies(PID_tracker_node slam_generate_messages_cpp)
add_dependencies(pid_control_node core_msgs_generate_messages_cpp)
add_dependencies(pid_control_node slam_generate_messages_cpp)

 target_link_libraries(PID_tracker_node
   ${catkin_LIBRARIES}
 )
 target_link_libraries(pub_vel
   ${catkin_LIBRARIES}
 )
 target_link_libraries(pid_control_node
   ${catkin_LIBRARIES}
 )

