cmake_minimum_required(VERSION 3.8)
project(imu_localization)

find_package(ament_cmake REQUIRED)
find_package(robot_localization REQUIRED)

install(
  DIRECTORY launch cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
