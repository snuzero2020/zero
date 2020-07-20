cmake_minimum_required(VERSION 2.8.3)
project(localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nmea_msgs
  message_generation
  cv_bridge
  cmake_modules
)

list(APPEND CMAK_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)

add_message_files(DIRECTORY msg FILES FloatStamp.msg Keyop.msg Data.msg Gps.msg Imu.msg Points.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs nmea_msgs)

catkin_package(CATKIN_DEPENDS message_runtime DEPENDS Eigen)

add_library(UnixtimeToSec lib/UnixtimeToSec.cpp)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(gps src/gps.cpp)
target_link_libraries(gps ${catkin_LIBRARIES})
add_dependencies(gps localization_generate_messages_cpp)

add_executable(imu src/imu.cpp)
target_link_libraries(imu ${catkin_LIBRARIES})
add_dependencies(imu localization_generate_messages_cpp)

add_executable(kalman_fusion src/kalman_fusion.cpp)
target_link_libraries(kalman_fusion ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(kalman_fusion localization_generate_messages_cpp)

add_executable(evaluator src/evaluator.cpp)
target_link_libraries(evaluator ${catkin_LIBRARIES})
add_dependencies(evaluator localization_generate_messages_cpp)

add_executable(imu_decoder src/imu_decoder.cpp)
target_link_libraries(imu_decoder ${catkin_LIBRARIES} UnixtimeToSec)
add_dependencies(imu_decoder localization_generate_messages_cpp)

catkin_install_python(PROGRAMS src/gps_decoder.py src/kalman_fusion_visualizer.py src/gps_marker.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
