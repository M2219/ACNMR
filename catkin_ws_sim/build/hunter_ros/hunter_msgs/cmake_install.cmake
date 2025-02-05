# Install script for directory: /root/hunter2_proj/catkin_ws_sim/src/hunter_ros/hunter_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/root/hunter2_proj/catkin_ws_sim/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hunter_msgs/msg" TYPE FILE FILES
    "/root/hunter2_proj/catkin_ws_sim/src/hunter_ros/hunter_msgs/msg/HunterMotorState.msg"
    "/root/hunter2_proj/catkin_ws_sim/src/hunter_ros/hunter_msgs/msg/HunterStatus.msg"
    "/root/hunter2_proj/catkin_ws_sim/src/hunter_ros/hunter_msgs/msg/HunterDriverState.msg"
    "/root/hunter2_proj/catkin_ws_sim/src/hunter_ros/hunter_msgs/msg/HunterBmsStatus.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hunter_msgs/cmake" TYPE FILE FILES "/root/hunter2_proj/catkin_ws_sim/build/hunter_ros/hunter_msgs/catkin_generated/installspace/hunter_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/root/hunter2_proj/catkin_ws_sim/devel/include/hunter_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/root/hunter2_proj/catkin_ws_sim/devel/share/roseus/ros/hunter_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/root/hunter2_proj/catkin_ws_sim/devel/share/common-lisp/ros/hunter_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/root/hunter2_proj/catkin_ws_sim/devel/share/gennodejs/ros/hunter_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/root/hunter2_proj/catkin_ws_sim/devel/lib/python3/dist-packages/hunter_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/root/hunter2_proj/catkin_ws_sim/devel/lib/python3/dist-packages/hunter_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/root/hunter2_proj/catkin_ws_sim/build/hunter_ros/hunter_msgs/catkin_generated/installspace/hunter_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hunter_msgs/cmake" TYPE FILE FILES "/root/hunter2_proj/catkin_ws_sim/build/hunter_ros/hunter_msgs/catkin_generated/installspace/hunter_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hunter_msgs/cmake" TYPE FILE FILES
    "/root/hunter2_proj/catkin_ws_sim/build/hunter_ros/hunter_msgs/catkin_generated/installspace/hunter_msgsConfig.cmake"
    "/root/hunter2_proj/catkin_ws_sim/build/hunter_ros/hunter_msgs/catkin_generated/installspace/hunter_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hunter_msgs" TYPE FILE FILES "/root/hunter2_proj/catkin_ws_sim/src/hunter_ros/hunter_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/root/hunter2_proj/catkin_ws_sim/build/hunter_ros/hunter_msgs/catkin_generated/installspace/hunter_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hunter_msgs/cmake" TYPE FILE FILES "/root/hunter2_proj/catkin_ws_sim/build/hunter_ros/hunter_msgs/catkin_generated/installspace/hunter_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hunter_msgs/cmake" TYPE FILE FILES
    "/root/hunter2_proj/catkin_ws_sim/build/hunter_ros/hunter_msgs/catkin_generated/installspace/hunter_msgsConfig.cmake"
    "/root/hunter2_proj/catkin_ws_sim/build/hunter_ros/hunter_msgs/catkin_generated/installspace/hunter_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hunter_msgs" TYPE FILE FILES "/root/hunter2_proj/catkin_ws_sim/src/hunter_ros/hunter_msgs/package.xml")
endif()

