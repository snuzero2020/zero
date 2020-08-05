cmake_minimum_required(VERSION 3.0.2)
project(pp_debug)

add_compile_options("-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  core_msgs
  message_filters
)


## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs core_msgs message_filters
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
)

# pp_debuger
add_executable(pp_debuger src/pp_debuger.cpp)

add_dependencies(pp_debuger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pp_debuger core_msgs_generate_messages_cpp )

target_link_libraries(pp_debuger  ${catkin_LIBRARIES} )
