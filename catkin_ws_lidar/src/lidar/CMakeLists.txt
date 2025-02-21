cmake_minimum_required(VERSION 3.0.2)
project(fakelidar)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  tf
)

catkin_package()

install(PROGRAMS
  scripts/fakelidar.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
