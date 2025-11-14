cmake_minimum_required(VERSION 3.0.2)
project(ugv_path_tracking)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  ugv_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf2_geometry_msgs tf2_ros ugv_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_core
  src/controllers/pure_pursuit_controller.cpp
  src/controllers/stanley_controller.cpp
  src/trajectory_utils.cpp
)
add_dependencies(${PROJECT_NAME}_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_core ${catkin_LIBRARIES})

add_executable(global_path_node src/nodes/global_path_node.cpp)
add_dependencies(global_path_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_path_node ${PROJECT_NAME}_core ${catkin_LIBRARIES})

add_executable(pure_pursuit_node src/nodes/pure_pursuit_node.cpp)
add_dependencies(pure_pursuit_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pure_pursuit_node ${PROJECT_NAME}_core ${catkin_LIBRARIES})

add_executable(stanley_node src/nodes/stanley_node.cpp)
add_dependencies(stanley_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(stanley_node ${PROJECT_NAME}_core ${catkin_LIBRARIES})

add_executable(tracking_evaluator_node src/nodes/tracking_evaluator_node.cpp)
add_dependencies(tracking_evaluator_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracking_evaluator_node ${PROJECT_NAME}_core ${catkin_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}_core
  global_path_node
  pure_pursuit_node
  stanley_node
  tracking_evaluator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
