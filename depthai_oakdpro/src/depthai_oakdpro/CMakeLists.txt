cmake_minimum_required(VERSION 3.10)
project(depthai_oakdpro LANGUAGES CXX)

# Use C++14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Define executable
add_executable(depthai_oakdpro_node src/depthai_oakdpro_node.cpp)
add_executable(sync_sanity_checker src/sync_sanity_checker.cpp)

# Link libraries
target_link_libraries(depthai_oakdpro_node
  depthai::core
  depthai_bridge::depthai_bridge  # Ensure depthai_bridge is linked
  ${OpenCV_LIBS}
)


# Add dependencies
ament_target_dependencies(depthai_oakdpro_node
  rclcpp
  sensor_msgs
  depthai_ros_msgs
  depthai_bridge
)

ament_target_dependencies(sync_sanity_checker rclcpp sensor_msgs)


# Install the node
install(TARGETS depthai_oakdpro_node  DESTINATION lib/${PROJECT_NAME})
install(TARGETS sync_sanity_checker DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)
# Ament package
ament_package()
