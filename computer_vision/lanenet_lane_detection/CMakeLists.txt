cmake_minimum_required(VERSION 3.0.2)
project(lanenet_lane_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV 3 REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs

	DEPENDS
	opencv_highgui opencv_imgproc
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(LaneNet_main src/LaneNet_seg_subscriber.cpp ${HEADERS} ${SOURCES})
add_dependencies(LaneNet_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(LaneNet_main ${catkin_LIBRARIES} ${OpenCV_LIBS})