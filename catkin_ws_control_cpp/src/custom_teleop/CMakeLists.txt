cmake_minimum_required(VERSION 3.0.2)
project(custom_teleop)

## Find catkin packages and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

## Find additional dependencies (Eigen, OsqpEigen)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

## Declare the catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES custom_teleop
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf
  DEPENDS EIGEN3 OsqpEigen
)

## Include directories
include_directories(
  include/custom_teleop
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OsqpEigen_INCLUDE_DIRS}
  /root/hunter2_proj/catkin_ws_sim/src/hunter_ros/hunter_base/include/hunter_base
)

## Add executable
add_executable(mpc_control_node
  src/main.cpp
  src/cubic_planner.cpp
  src/mpc_controller.cpp
  src/mpc_utils.cpp
)

## Link libraries
target_link_libraries(mpc_control_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
)

## Install targets
install(TARGETS mpc_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
