cmake_minimum_required(VERSION 2.8.3)
project(slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  nmea_msgs
  nav_msgs
  message_generation
  cv_bridge
  message_filters
  cmake_modules #for eigen used in sensing/kalman_fusion
)
#for eigen used in sensing/kalman_fusion
list(APPEND CMAK_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
#

find_package(OpenCV REQUIRED)

add_message_files(
  DIRECTORY msg 
  FILES 
  Data.msg Gps.msg Imu.msg Lidar.msg Cluster.msg Pixel.msg GlobalPathPoint.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs nmea_msgs)

catkin_package(CATKIN_DEPENDS message_runtime DEPENDS Eigen)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

#add_executable for sensing
add_executable(gps_decoder src/sensing/gps_decoder.cpp lib/UTM.cpp)
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

#add_executable(gps_subscriber_node src/mapping/gps_subscriber_node.cpp lib/UTM.cpp lib/XYToPixel.cpp)
#target_link_libraries(gps_subscriber_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(gps_subscriber_node slam_generate_messages_cpp)

#add_executable(Map_Cutter src/mapping/main.cpp src/mapping/MapCutter.cpp lib/UTM.cpp lib/XYToPixel.cpp)
#target_link_libraries(Map_Cutter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(Map_Cutter slam_generate_messages_cpp)

add_executable(map_cost_spreader src/mapping/map_cost_spreader.cpp)
target_link_libraries(map_cost_spreader ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable for clustering

#add_executable for localmap publishing
#add_executable(Localmap_Publisher src/localmap_pubnode/localmap_publisher.cpp src/mapping/MapCutter.cpp lib/XYToPixel.cpp)
#target_link_libraries(Localmap_Publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(Localmap_Publisher slam_generate_messages_cpp)

#add_executable for localpath publishing
add_executable(locpath_publisher src/locpath_pubnode/locpath_publisher.cpp lib/XYToPixel.cpp)
target_link_libraries(locpath_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(locpath_publisher slam_generate_messages_cpp)

#add_executable for minimap publisher
add_executable(minimap_publisher src/mini_map/minimap_publisher.cpp lib/XYToPixel.cpp)
target_link_libraries(minimap_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(minimap_publisher slam_generate_messages_cpp)

#add_executable for BGRmap_to_costmap
add_executable(BGRmap_to_costmap src/BGRmap_to_costmap/main.cpp src/BGRmap_to_costmap/BGRmapToCostmap.cpp)
target_link_libraries(BGRmap_to_costmap ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(BGRmap_to_costmap)

#add_executable for global_path
add_executable(globpath_generator src/global_path/globpath_generator.cpp lib/XYToPixel.cpp)
target_link_libraries(globpath_generator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(globpath_generator slam_generate_messages_cpp)

add_executable(object_detector src/clustering/object_detector.cpp)
target_link_libraries(object_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(object_detector slam_generate_messages_cpp)

#add executable for local_costmap_publisher
add_executable(local_costmap_publisher src/mapping/local_costmap_publisher.cpp lib/XYToPixel.cpp)
target_link_libraries(local_costmap_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(local_costmap_publisher slam_generate_messages_cpp)

#add_executable for local_costmap_publisher
add_executable(visualizer src/visualizer/visualizer.cpp lib/XYToPixel.cpp)
target_link_libraries(visualizer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(visualizer slam_generate_messages_cpp)

add_executable(lidar_pose_estimator src/clustering/lidar_pose_estimator.cpp)
target_link_libraries(lidar_pose_estimator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(lidar_pose_estimator slam_generate_messages_cpp)

add_executable(obstacle_image src/clustering/obstacle_image.cpp)
target_link_libraries(obstacle_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(obstacle_image slam_generate_messages_cpp)

add_executable(cone_detection src/clustering/find_cone.cpp)
target_link_libraries(cone_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(cone_detection slam_generate_messages_cpp)

add_executable(gps_collector src/mapping/gps_collector.cpp)
target_link_libraries(gps_collector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(gps_collector slam_generate_messages_cpp)

add_executable(convert_XY_To_Pixel src/mapping/convert_XY_To_Pixel.cpp)
target_link_libraries(convert_XY_To_Pixel ${catkin_LIBRARIES})
add_dependencies(convert_XY_To_Pixel slam_generate_messages_cpp)

add_executable(sector_publisher src/mapping/sector_publisher.cpp)
target_link_libraries(sector_publisher ${catkin_LIBRARIES})
add_dependencies(sector_publisher slam_generate_messages_cpp)

add_executable(local_path_publisher src/global_path/local_path_publisher.cpp)
target_link_libraries(local_path_publisher ${catkin_LIBRARIES})
add_dependencies(local_path_publisher slam_generate_messages_cpp)

add_executable(global_path_generator src/global_path/global_path_generator.cpp)
target_link_libraries(global_path_generator ${catkin_LIBRARIES})
add_dependencies(global_path_generator slam_generate_messages_cpp)


catkin_install_python(PROGRAMS 
  src/sensing/gps_decoder.py src/sensing/imu_visualizer.py src/sensing/kalman_fusion_visualizer.py src/sensing/lidar_decoder.py
  src/mapping/transform_finder.py
  src/clustering/clustering.py src/clustering/object_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
