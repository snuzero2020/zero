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
  Data.msg Gps.msg Imu.msg Lidar.msg LidarPoint.msg Cluster.msg Pixel.msg GlobalPathPoint.msg 
  Yolomaster.msg Yoloinfo.msg Clustermaster.msg imgCluster.msg Clusters.msg
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

add_executable(global_visual src/visualizer/global_path_visualizer.cpp lib/XYToPixel.cpp)
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

add_executable(color_velocity src/map_layer/color_velocity.cpp)
target_link_libraries(color_velocity ${catkin_LIBRARIES})
add_dependencies(color_velocity slam_generate_messages_cpp)

add_executable(velocity_publisher src/map_layer/velocity_publisher.cpp lib/XYToPixel.cpp)
target_link_libraries(velocity_publisher ${catkin_LIBRARIES})
add_dependencies(velocity_publisher slam_generate_messages_cpp)

##add_executable for obstacle_cost_map
add_executable(obstacle_detector src/obstacle_cost_map/obstacle_detector.cpp lib/XYToPixel.cpp)
target_link_libraries(obstacle_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(obstacle_detector slam_generate_messages_cpp)

#add_executable(lidar_pose_estimator src/obstacle_cost_map/lidar_pose_estimator.cpp)
#target_link_libraries(lidar_pose_estimator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
#add_dependencies(lidar_pose_estimator slam_generate_messages_cpp)

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

add_executable(pp_visualizer src/visualizer/pp_visualizer.cpp lib/XYToPixel.cpp)
target_link_libraries(pp_visualizer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(pp_visualizer slam_generate_messages_cpp)

add_executable(obstacle_image src/obstacle_cost_map/obstacle_image.cpp)
target_link_libraries(obstacle_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(obstacle_image slam_generate_messages_cpp)

add_executable(obstacle_costmap_publisher src/obstacle_cost_map/find_cone.cpp)
target_link_libraries(obstacle_costmap_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(obstacle_costmap_publisher slam_generate_messages_cpp)

#add_executable(yolo_sender src/obstacle_cost_map/send_yolo.cpp)
#target_link_libraries(yolo_sender ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(yolo_sender slam_generate_messages_cpp)

#add_executable(yolo_lidar src/obstacle_cost_map/yolo_lidar_fusion.cpp)
#target_link_libraries(yolo_lidar ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(yolo_lidar slam_generate_messages_cpp)

#add_executable(calib_matrix src/obstacle_cost_map/calib_mtx.cpp)
#target_link_libraries(calib_matrix ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(calib_matrix slam_generate_messages_cpp)

#add_executable(lidar_visualizer src/obstacle_cost_map/lidar_visualizer.cpp)
#target_link_libraries(lidar_visualizer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(lidar_visualizer slam_generate_messages_cpp)

#add_executable(video_capture src/obstacle_cost_map/video_capture.cpp)
#target_link_libraries(video_capture ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(video_capture slam_generate_messages_cpp)


catkin_install_python(PROGRAMS 
  src/map_calibration/transform_finder.py
  src/sensing/lidar_decoder.py
  src/visualizer/imu_visualizer.py 
  src/visualizer/kalman_fusion_visualizer.py
  src/obstacle_cost_map/object_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
