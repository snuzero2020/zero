cmake_minimum_required(VERSION 2.8)
project(Lane_Detection)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

file(GLOB SOURCES "src/*.cpp")
add_executable(Lane_Detection ${SOURCES})
target_link_libraries(Lane_Detection ${OpenCV_LIBS})

