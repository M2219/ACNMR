cmake_minimum_required(VERSION 3.0.2)
project(slam)

## Find required ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  map_server
  amcl
)

## Declare the catkin package
catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf amcl map_server
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Launch Files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install Config Files
install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
)

## Install Stage World Files
#install(DIRECTORY stage/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/stage
#)

## Install Package XML
install(FILES package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


## Add Executables (Uncomment if you have C++ nodes)
# add_executable(my_node src/my_node.cpp)
# target_link_libraries(my_node ${catkin_LIBRARIES})
