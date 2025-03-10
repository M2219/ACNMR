cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(slam_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
)

link_directories(
  include
)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV)

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge message_filters ORB_SLAM3 Pangolin OpenCV)

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-node.cpp
  src/type_conversion.cpp
  src/orb_slam3_interface.cpp
  src/time_profiler.cpp
)

ament_target_dependencies(stereo rclcpp sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs tf2_eigen slam_msgs std_srvs cv_bridge ORB_SLAM3 Pangolin OpenCV)

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
  src/type_conversion.cpp
  src/orb_slam3_interface.cpp
  src/time_profiler.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs tf2_eigen slam_msgs std_srvs cv_bridge ORB_SLAM3 Pangolin OpenCV)

add_executable(imu_buffer_checker src/imu_buffer_checker.cpp)

ament_target_dependencies(imu_buffer_checker
  rclcpp
  sensor_msgs
  std_msgs
)

install(DIRECTORY launch params
DESTINATION share/${PROJECT_NAME}
)

install(TARGETS imu_buffer_checker mono rgbd stereo stereo-inertial DESTINATION lib/${PROJECT_NAME})

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()

