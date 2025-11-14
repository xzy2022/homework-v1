cmake_minimum_required(VERSION 3.0.2)
project(ugv_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  std_msgs
)

add_message_files(
  FILES
  ControlCommand.msg
  Trajectory.msg
  TrajectoryPoint.msg
  TrackingPerformance.msg
  VehicleState.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    nav_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
