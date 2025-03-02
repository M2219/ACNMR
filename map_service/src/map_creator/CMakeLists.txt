cmake_minimum_required(VERSION 3.8)
project(map_creator)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
# Include directories
include_directories(
  include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install configuration files
install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)

# Install map files
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_package()
