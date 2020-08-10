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
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
#

find_package(OpenCV REQUIRED)

add_message_files(
  DIRECTORY msg 
  FILES 
  Data.msg Gps.msg Imu.msg Lidar.msg Cluster.msg Clusters.msg Pixel.msg GlobalPathPoint.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs nmea_msgs)

catkin_package(CATKIN_DEPENDS message_runtime DEPENDS Eigen)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

##add_executable for cost_map
add_executable(local_costmap_publisher src/cost_map/local_costmap_publisher.cpp lib/XYToPixel.cpp)
target_link_libraries(local_costmap_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(local_costmap_publisher slam_generate_messages_cpp)

add_executable(map_cost_spreader src/cost_map/map_cost_spreader.cpp)
target_link_libraries(map_cost_spreader ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


##add_executable for global_path
add_executable(outlier_eliminator src/global_path/auto_outlier_eliminator.cpp)
target_link_libraries(outlier_eliminator ${catkin_LIBRARIES})
add_dependencies(outlier_eliminator slam_generate_messages_cpp)

add_executable(global_path_generator src/global_path/global_path_generator.cpp)
target_link_libraries(global_path_generator ${catkin_LIBRARIES})
add_dependencies(global_path_generator slam_generate_messages_cpp)

add_executable(global_path_graph_generator src/global_path/global_path_graph_generator.cpp)
target_link_libraries(global_path_graph_generator ${catkin_LIBRARIES})
add_dependencies(global_path_graph_generator slam_generate_messages_cpp)

add_executable(local_path_publisher src/global_path/local_path_publisher.cpp)
target_link_libraries(local_path_publisher ${catkin_LIBRARIES})
add_dependencies(local_path_publisher slam_generate_messages_cpp)

add_executable(global_visual src/global_path/global_path_visualizer.cpp lib/XYToPixel.cpp)
target_link_libraries(global_visual ${catkin_LIBRARIES})
add_dependencies(global_visual slam_generate_messages_cpp)

##add_executable for map_calibration
add_executable(gps_collector src/map_calibration/gps_collector.cpp)
target_link_libraries(gps_collector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(gps_collector slam_generate_messages_cpp)

##add_executable for map_layer
add_executable(sector_publisher src/map_layer/sector_publisher.cpp lib/XYToPixel.cpp)
target_link_libraries(sector_publisher ${catkin_LIBRARIES})
add_dependencies(sector_publisher slam_generate_messages_cpp)

##add_executable for obstacle_cost_map
add_executable(object_detector src/obstacle_cost_map/object_detector.cpp)
target_link_libraries(object_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(object_detector slam_generate_messages_cpp)

add_executable(lidar_pose_estimator src/obstacle_cost_map/lidar_pose_estimator.cpp)
target_link_libraries(lidar_pose_estimator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(lidar_pose_estimator slam_generate_messages_cpp)

##add_executable for sensing
add_executable(gps_decoder src/sensing/gps_decoder.cpp lib/UTM.cpp)
target_link_libraries(gps_decoder ${catkin_LIBRARIES})
add_dependencies(gps_decoder slam_generate_messages_cpp)

add_executable(imu_decoder src/sensing/imu_decoder.cpp lib/UnixtimeToSec.cpp)
target_link_libraries(imu_decoder ${catkin_LIBRARIES})
add_dependencies(imu_decoder slam_generate_messages_cpp)

add_executable(kalman_fusion src/sensing/kalman_fusion.cpp)
target_link_libraries(kalman_fusion ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(kalman_fusion slam_generate_messages_cpp)

##add_executable for visualizer
add_executable(gps_marker src/visualizer/gps_marker.cpp lib/XYToPixel.cpp)
target_link_libraries(gps_marker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(gps_marker slam_generate_messages_cpp)

add_executable(minimap_publisher src/visualizer/minimap_publisher.cpp lib/XYToPixel.cpp)
target_link_libraries(minimap_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(minimap_publisher slam_generate_messages_cpp)

add_executable(slam_visualizer src/visualizer/slam_visualizer.cpp lib/XYToPixel.cpp)
target_link_libraries(slam_visualizer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(slam_visualizer slam_generate_messages_cpp)

add_executable(visualizer src/visualizer/visualizer.cpp lib/XYToPixel.cpp)
target_link_libraries(visualizer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(visualizer slam_generate_messages_cpp)

catkin_install_python(PROGRAMS 
  src/map_calibration/transform_finder.py
  src/sensing/lidar_decoder.py
  src/visualizer/imu_visualizer.py 
  src/visualizer/kalman_fusion_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
