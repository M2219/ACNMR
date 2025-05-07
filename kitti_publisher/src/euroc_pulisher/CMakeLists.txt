cmake_minimum_required(VERSION 3.10)
project(euroc_publisher LANGUAGES CXX)

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
find_package(rosbag2_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
# oak pro gpu
find_package(CUDA REQUIRED)
set(TENSORRT_LIBS nvinfer nvonnxparser nvinfer_plugin)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  /usr/include/x86_64-linux-gnu

)

# Define executable
add_executable(euroc_publisher_cuda_node src/euroc_publisher_cuda_node.cpp)
add_executable(sync_sanity_checker src/sync_sanity_checker.cpp)

target_link_libraries(euroc_publisher_cuda_node
  ${OpenCV_LIBS}
  ${TENSORRT_LIBS}
  ${CUDA_LIBRARIES}
  /usr/lib/x86_64-linux-gnu

)

ament_target_dependencies(euroc_publisher_cuda_node
  rclcpp
  sensor_msgs
  OpenCV
  rosbag2_cpp
  rosidl_typesupport_cpp
  geometry_msgs
  tf2_ros
  cv_bridge
)

ament_target_dependencies(sync_sanity_checker rclcpp sensor_msgs)


# Install the node
install(TARGETS euroc_publisher_cuda_node  DESTINATION lib/${PROJECT_NAME})
install(TARGETS sync_sanity_checker DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)
# Ament package
ament_package()
