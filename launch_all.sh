#!/bin/bash

# Source each workspace and launch in the background
source /root/hunter2_proj/catkin_ws_sim/devel/setup.bash && roslaunch hunter_bringup hunter_robot_base.launch &
source /root/hunter2_proj/catkin_ws_lidar/devel/setup.bash && roslaunch fakelidar lidar_node.launch &
source /root/hunter2_proj/catkin_ws_slam/devel/setup.bash && roslaunch slam slam_node.launch &
source /root/hunter2_proj/catkin_ws_map/devel/setup.bash && roslaunch map_creator map_node.launch &
rosrun rqt_multiplot rqt_multiplot &

# Keep script running
wait
