cmake_minimum_required(VERSION 2.8.3)
project(localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  cv_bridge
)

find_package(cmake_modules REQUIRED)
list(APPEND CMAK_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
add_message_files(DIRECTORY msg FILES FloatStamp.msg Keyop.msg Data.msg Gps.msg Imu.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime DEPENDS Eigen)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(camera_motor src/camera_motor.cpp)
target_link_libraries(camera_motor ${catkin_LIBRARIES})
add_dependencies(camera_motor localization_generate_messages_cpp)

add_executable(motor_angle src/motor_angle.cpp)
target_link_libraries(motor_angle ${catkin_LIBRARIES})
add_dependencies(motor_angle localization_generate_messages_cpp)

add_executable(keyop_control src/keyop_control.cpp)
target_link_libraries(keyop_control ${catkin_LIBRARIES})
add_dependencies(keyop_control localization_generate_messages_cpp)

add_executable(velocity_from_keyop src/velocity_from_keyop.cpp)
target_link_libraries(velocity_from_keyop ${catkin_LIBRARIES})
add_dependencies(velocity_from_keyop localization_generate_messages_cpp)

add_executable(global_velocity src/global_velocity.cpp)
target_link_libraries(global_velocity ${catkin_LIBRARIES})
add_dependencies(global_velocity localization_generate_messages_cpp)

add_executable(car_position src/car_position.cpp)
target_link_libraries(car_position ${catkin_LIBRARIES})
add_dependencies(car_position localization_generate_messages_cpp)

add_executable(depth_camera_position src/depth_camera_position.cpp)
target_link_libraries(depth_camera_position ${catkin_LIBRARIES})
add_dependencies(depth_camera_position localization_generate_messages_cpp)

add_executable(show_car_position src/show_car_position.cpp)
find_package(OpenCV)
target_link_libraries(show_car_position ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(show_car_position localization_generate_messages_cpp)

add_executable(simulation src/simulation.cpp)
target_link_libraries(simulation ${catkin_LIBRARIES})
add_dependencies(simulation localization_generate_messages_cpp)

add_executable(gps src/gps.cpp)
target_link_libraries(gps ${catkin_LIBRARIES})
add_dependencies(gps localization_generate_messages_cpp)

add_executable(imu src/imu.cpp)
target_link_libraries(imu ${catkin_LIBRARIES})
add_dependencies(imu localization_generate_messages_cpp)

add_executable(kalman_fusion src/kalman_fusion.cpp)
target_link_libraries(kalman_fusion ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(kalman_fusion localization_generate_messages_cpp)

add_executable(evaluator src/evaluator.cpp)
target_link_libraries(evaluator ${catkin_LIBRARIES})
add_dependencies(evaluator localization_generate_messages_cpp)

add_executable(imu_decoder src/imu_decoder.cpp)
target_link_libraries(imu_decoder ${catkin_LIBRARIES})
add_dependencies(imu_decoder localization_generate_messages_cpp)
