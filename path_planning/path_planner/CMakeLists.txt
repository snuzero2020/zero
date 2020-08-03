cmake_minimum_required(VERSION 3.0.2)
project(path_planner)

add_compile_options("-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  core_msgs
)

find_package(OpenCV 3 REQUIRED)

## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
include/${PROJECT_NAME}
)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(rrt_star src/rrt_star.cpp)

add_library(goal_decision src/goal_decision.cpp)

# mission_recognizer_node
add_executable(mission_recognizer_node src/mission_recognizer.cpp)

add_dependencies(mission_recognizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mission_recognizer_node  ${catkin_LIBRARIES} )

# path_planner_node
add_executable(path_planner_node src/path_planner_node.cpp)

add_dependencies(path_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(path_planner_node core_msgs_generate_messages_cpp)

target_link_libraries(path_planner_node  ${catkin_LIBRARIES} )

target_link_libraries(path_planner_node  rrt_star )

target_link_libraries(path_planner_node  goal_decision )

target_link_libraries(path_planner_node  ${OpenCV_LIBS})

set_target_properties(path_planner_node PROPERTIES COMPILE_FLAGS "-O3")

# sector_info_pub_node
add_executable(sector_info_pub_node src/sector_info_pub_test.cpp)

add_dependencies(sector_info_pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(sector_info_pub_node  ${catkin_LIBRARIES} )

set_target_properties(sector_info_pub_node PROPERTIES COMPILE_FLAGS "-O3")

# path_listener_node
add_executable(path_listener_node src/path_listener_node.cpp)

add_dependencies(path_listener_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(path_listener_node  ${catkin_LIBRARIES} )

set_target_properties(path_listener_node PROPERTIES COMPILE_FLAGS "-O3")

# cost_map_generator_node
add_executable(cost_map_generator_node src/cost_map_generator_node.cpp)

add_dependencies(cost_map_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cost_map_generator_node  ${catkin_LIBRARIES} )

set_target_properties(cost_map_generator_node PROPERTIES COMPILE_FLAGS "-O3")
