cmake_minimum_required(VERSION 3.8)
project(imu_localization)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_localization REQUIRED)

# Install launch files (*.py and *.xml)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.py"
  PATTERN "*.xml"
)

# Install config files
install(
  DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)

ament_package()
