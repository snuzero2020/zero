cmake_minimum_required(VERSION 2.8.3)
project(gps_subscriber)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nmea_msgs
  roscpp
  std_msgs
  localization
)

find_package(OpenCV REQUIRED)

generate_messages(
   DEPENDENCIES
   nmea_msgs
   std_msgs
)

catkin_package(
  LIBRARIES gps_subscriber
  CATKIN_DEPENDS nmea_msgs roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(gps_subscriber_node src/gps_subscriber_node.cpp lib/UTM.cpp lib/XYToPixel.cpp)
add_dependencies(gps_subscriber_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_subscriber_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)
