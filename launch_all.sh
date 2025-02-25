#!/bin/bash

# Source each workspace and launch in the background
cd catkin_ws_robot
source devel/setup.bash && roslaunch hunter_bringup hunter_robot_base.launch &
cd ..
cd catkin_ws_lidar
source devel/setup.bash && roslaunch fakelidar lidar_node.launch &
cd ..
cd catkin_ws_slam
source devel/setup.bash && roslaunch slam slam_node.launch &
cd ..
cd catkin_ws_map
source devel/setup.bash && roslaunch map_creator map_node.launch &
cd ..
cd hybrid_a_star_ws
source devel/setup.bash && roslaunch hybrid_a_star run_hybrid_a_star.launch &
cd ..
rosrun rqt_multiplot rqt_multiplot &

# Keep script running
wait
