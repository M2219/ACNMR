
cmake_minimum_required(VERSION 3.8)
project(custom_teleop)

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
## Include directories
include_directories(
  include/custom_teleop
  ${EIGEN3_INCLUDE_DIR}
  ${OsqpEigen_INCLUDE_DIRS}
  /root/hunter2_proj/catkin_ws_sim/src/hunter_ros/hunter_base/include/hunter_base
)

## Add executable
add_executable(mpc_control_node
  src/main.cpp
  src/mpc_controller.cpp
  src/mpc_utils.cpp
  src/cubic_planner.cpp
)

## Link libraries
target_link_libraries(mpc_control_node
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
)

## Specify dependencies for the target
ament_target_dependencies(mpc_control_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  Eigen3
  OsqpEigen
  tf2_geometry_msgs
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

## Install executable
install(TARGETS mpc_control_node
  DESTINATION lib/${PROJECT_NAME}
)

## Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
)

## Export package
ament_export_dependencies(rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros Eigen3 OsqpEigen)
ament_export_include_directories(include)

## Finalize package
ament_package()
