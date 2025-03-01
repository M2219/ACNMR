cmake_minimum_required(VERSION 3.10)
project(hybrid_a_star)

message(STATUS "Hybrid A* Planner")
find_package(glog REQUIRED)
message(STATUS "Author: Zhang Zhimeng")

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ADDITIONAL_CXX_FLAGS "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAGS}")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ../../../mpc_control_cpp_auto/src/custom_teleop/include/custom_teleop
)

include(cmake/glog.cmake)
include_directories(${GLOG_INCLUDE_DIRS})
# Eigen
include(cmake/FindEigen.cmake)
include_directories(${EIGEN_INCLUDE_DIR})
# Source files
set(SOURCES
    src/rs_path.cpp
    src/costmap_subscriber.cpp
    src/goal_pose_subscriber.cpp
    src/init_pose_subscriber.cpp
    src/hybrid_a_star.cpp
    src/hybrid_a_star_flow.cpp
)

# Create library
add_library(hybrid_a_star_lib SHARED
    src/costmap_subscriber.cpp
    src/goal_pose_subscriber.cpp
    src/hybrid_a_star.cpp
    src/hybrid_a_star_flow.cpp
    src/init_pose_subscriber.cpp
    src/rs_path.cpp
)

ament_target_dependencies(hybrid_a_star_lib
    rclcpp
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    Eigen3
    tf2_geometry_msgs

)

# Executable for running the node
add_executable(hybrid_astar_launch app/run_hybrid_astar.cpp)
target_link_libraries(hybrid_astar_launch
    hybrid_a_star_lib
    glog::glog
)
ament_target_dependencies(hybrid_astar_launch
    rclcpp
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

# Install shared library correctly
install(
    TARGETS hybrid_a_star_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
)

# Install executables correctly
install(
    TARGETS hybrid_astar_launch
    DESTINATION lib/${PROJECT_NAME}  # This ensures the binary goes to 'lib/hybrid_a_star/'
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

ament_package()
