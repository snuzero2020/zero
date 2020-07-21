cmake_minimum_required(VERSION 2.8.3)
project(slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  nmea_msgs
  message_generation
  cv_bridge
  cmake_modules #for eigen used in sensing/kalman_fusion
)
#for eigen used in sensing/kalman_fusion
list(APPEND CMAK_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
#

add_message_files(
  DIRECTORY msg 
  FILES 
  Data.msg Gps.msg Imu.msg Lidar.msg Cluster.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs nmea_msgs)

catkin_package(CATKIN_DEPENDS message_runtime DEPENDS Eigen)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

#add_executable for sensing
add_executable(gps_decoder src/sensing/imu_decoder.cpp lib/UTM.cpp)
target_link_libraries(gps_decoder ${catkin_LIBRARIES})
add_dependencies(gps_decoder slam_generate_messages_cpp)

add_executable(imu_decoder src/sensing/imu_decoder.cpp lib/UnixtimeToSec.cpp)
target_link_libraries(imu_decoder ${catkin_LIBRARIES})
add_dependencies(imu_decoder slam_generate_messages_cpp)

add_executable(kalman_fusion src/sensing/kalman_fusion.cpp)
target_link_libraries(kalman_fusion ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(kalman_fusion slam_generate_messages_cpp)

#add_executable for mapping
add_executable(gps_marker src/mapping/gps_marker.cpp lib/XYToPixel.cpp)
target_link_libraries(gps_marker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(gps_marker slam_generate_messages_cpp)

add_executable(gps_subscriber_node src/mapping/gps_subscriber_node.cpp lib/UTM.cpp lib/XYToPixel.cpp)
target_link_libraries(gps_subscriber_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(gps_subscriber_node slam_generate_messages_cpp)

add_executable(map_cutter src/mapping/main.cpp src/mapping/map_cutter.cpp lib/UTM.cpp lib/XYToPixel.cpp)
target_link_libraries(map_cutter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(map_cutter slam_generate_messages_cpp)

#add_executable for clustering

catkin_install_python(PROGRAMS 
  src/sensing/gps_decoder.py src/sensing/imu_visualizer.py src/sensing/kalman_fusion_visualizer.py src/sensing/lidar_decoder.py
  src/clustering/clustering.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
