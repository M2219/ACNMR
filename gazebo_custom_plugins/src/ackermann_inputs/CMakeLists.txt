cmake_minimum_required(VERSION 3.5)
project(ackermann_inputs)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED) # Important for Gazebo plugins

# Include directories
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add the plugin
add_library(custom_ackermann_drive_plugin SHARED src/ackermann_drive.cpp)

# Link against required libraries
target_link_libraries(custom_ackermann_drive_plugin
  ${GAZEBO_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
)

# Add dependencies to ensure proper build order
ament_target_dependencies(custom_ackermann_drive_plugin
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  gazebo_ros
)

# Install the plugin
install(TARGETS custom_ackermann_drive_plugin
  DESTINATION lib/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(
  tf2
  tf2_ros
  tf2_geometry_msgs
  gazebo_ros
  rclcpp
)

ament_package()


