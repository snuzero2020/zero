cmake_minimum_required(VERSION 2.8.3)
project(core_msgs)

find_package(catkin REQUIRED
  message_generation
  geometry_msgs
  sensor_msgs
  std_msgs
)

add_message_files( DIRECTORY msg
  FILES
  VehicleState.msg
  Control.msg
  Curvature.msg
  MissionState.msg
  LightState.msg
  MotionState.msg
  VelocityLevel.msg
  ImuSpeed.msg
  ActiveNode.msg
  CenterLine.msg
  Location.msg
  UInt32WithHeader.msg
)

generate_messages(DEPENDENCIES   geometry_msgs  sensor_msgs   std_msgs )

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs std_msgs)

