cmake_minimum_required(VERSION 2.8.3)
project(map_cutter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(OpenCV REQUIRED)

catkin_package(
  LIBRARIES map_cutter
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(map_cutter src/main.cpp src/map_cutter.cpp lib/UTM.cpp lib/XYToPixel.cpp)
add_dependencies(map_cutter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_cutter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
