cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODUlE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)

add_message_files(DIRECTORY msg FILES Gps.msg Imu.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime DEPENDS Eigen)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(kalman_fusion src/kalman_fusion.cpp)
target_link_libraries(kalman_fusion ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(kalman_fusion sensor_fusion_generate_messages_cpp)

