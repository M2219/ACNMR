#!/bin/bash

# Source each workspace and launch in the background
cd simulated_robot
source install/setup.bash && ros2 launch hunter_base hunter_base.launch.py &
cd ..
cd lidar_sim
source install/setup.bash && ros2 launch fakelidar lidar_node.launch.py &
cd ..
cd slam_sim
source install/setup.bash && ros2 launch slam slam_node.launch.py &
cd ..
cd map_service
source install/setup.bash && ros2 launch map_creator map_node.launch.py &
cd ..
#cd hybrid_a_star_ws
#source install/setup.bash && ros2 launch hybrid_a_star hybrid_a_star_launch_file.launch.xml &
#cd ..
#ros2 run rqt_plot rqt_plot &
# Keep script running
wait
