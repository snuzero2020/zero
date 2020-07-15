cmake_minimum_required(VERSION 3.0.2)
project(path_planner)

add_compile_options("-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
include/${PROJECT_NAME}
)

add_library(rrt_star src/rrt_star.cpp)

add_executable(path_planner_node src/path_planner_node.cpp)

add_dependencies(path_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(path_planner_node  ${catkin_LIBRARIES} )

target_link_libraries(path_planner_node  rrt_star )

set_target_properties(path_planner_node PROPERTIES COMPILE_FLAGS "-O3")


add_executable(path_listener_node src/path_listener_node.cpp)

add_dependencies(path_listener_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(path_listener_node  ${catkin_LIBRARIES} )

set_target_properties(path_listener_node PROPERTIES COMPILE_FLAGS "-O3")


add_executable(cost_map_generator_node src/cost_map_generator_node.cpp)

add_dependencies(cost_map_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cost_map_generator_node  ${catkin_LIBRARIES} )

set_target_properties(cost_map_generator_node PROPERTIES COMPILE_FLAGS "-O3")

