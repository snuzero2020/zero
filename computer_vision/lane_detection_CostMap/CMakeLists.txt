cmake_minimum_required(VERSION 2.8)
project(lane_detection)
	
find_package(catkin REQUIRED COMPONENTS
		roscpp
		std_msgs
		nav_msgs
		geometry_msgs
)

find_package(OpenCV 3 REQUIRED)
#find_package(Eigen REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs

	DEPENDS
	opencv_highgui opencv_imgproc
)

#catkin_package(CATKIN_DEPENDS message_runtime DEPENDS Eigen)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
#include_directories(${Eigen_INCLUDE_DIRS})
include_directories(include)


add_library(func_util src/func_util.cpp)
add_library(lane_class src/lane_class.cpp)
add_library(lane_fit_sliding_window src/lane_fit_sliding_window.cpp)
add_library(lane_math src/lane_math.cpp)

set (SOURCES 
 ${CMAKE_CURRENT_SOURCE_DIR}/src/func_util.cpp
 ${CMAKE_CURRENT_SOURCE_DIR}/src/lane_class.cpp
 ${CMAKE_CURRENT_SOURCE_DIR}/src/lane_fit_sliding_window.cpp
 ${CMAKE_CURRENT_SOURCE_DIR}/src/lane_math.cpp
)


set (HEADERS
 ${CMAKE_CURRENT_SOURCE_DIR}/include/func_util.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/lane_class.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/lane_fit_sliding_window.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/lane_math.h
)

add_executable(cv_main src/main.cpp ${HEADERS} ${SOURCES})
add_dependencies(cv_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cv_main ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(cone_detection slam_generate_messages_cpp)
