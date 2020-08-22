cmake_minimum_required(VERSION 3.0.2)
project(lanenet_lane_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  cv_bridge
)

find_package(OpenCV 3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

add_message_files(
  FILES
  lanenet_msg.msg
  lanenet_clus_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge

	DEPENDS
	opencv_highgui opencv_imgproc
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)

add_library(multi_img_utils src/multi_img_utils.cpp)

set (SOURCES 
 ${CMAKE_CURRENT_SOURCE_DIR}/src/multi_img_utils.cpp
)

set (HEADERS
 ${CMAKE_CURRENT_SOURCE_DIR}/include/multi_img_utils.h
)

add_executable(lanenet_seg_subscriber src/LaneNet_seg_subscriber.cpp ${HEADERS} ${SOURCES})
add_dependencies(lanenet_seg_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lanenet_seg_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#set_target_properties(lanenet_seg_subscriber PROPERTIES COMPILE_FLAGS "-O2")

add_executable(lanenet_cluster_subscriber src/Lanenet_cluster_sub.cpp ${HEADERS} ${SOURCES})
add_dependencies(lanenet_cluster_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lanenet_cluster_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#set_target_properties(lanenet_cluster_subscriber PROPERTIES COMPILE_FLAGS "-O2")