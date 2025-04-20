cmake_minimum_required(VERSION 3.10)
project(disparity_oakdpro)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# CUDA
find_package(CUDA REQUIRED)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Torch paths (adjust if necessary)
set(Torch_DIR "/usr/local/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)

set(TENSORRT_LIBS nvinfer nvonnxparser nvinfer_plugin)


# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  /usr/include/x86_64-linux-gnu
)

# Source file
add_executable(disparity_oakdpro_node src/disparity_oakdpro_node.cpp)
add_executable(disparity_oakdpro_trt_node src/disparity_oakdpro_trt_node.cpp)

# Link libraries
target_link_libraries(disparity_oakdpro_node
  ${OpenCV_LIBS}
  ${TORCH_LIBRARIES}
)

target_link_libraries(disparity_oakdpro_trt_node
  ${OpenCV_LIBS}
  ${TENSORRT_LIBS}
  ${CUDA_LIBRARIES}
  /usr/lib/x86_64-linux-gnu
)

# Torch requires these flags
target_compile_definitions(disparity_oakdpro_node PRIVATE -D_GLIBCXX_USE_CXX11_ABI=1)

ament_target_dependencies(disparity_oakdpro_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

ament_target_dependencies(disparity_oakdpro_trt_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

# Install the node executable
install(TARGETS
  disparity_oakdpro_node
  disparity_oakdpro_trt_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files (e.g., model .pt file)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
