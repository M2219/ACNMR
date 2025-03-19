#!/bin/bash

sleep 1
LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d rviz_navigation.rviz &

# Source each workspace and launch in the background
cd simulated_robot
#cd robot_interface
source install/setup.bash && ros2 launch hunter_base hunter_base.launch.py &
cd ..

sleep 5
cd map_service
source install/setup.bash && ros2 launch map_creator map_node.launch.py &
cd ..


sleep 2
cd lidar_sim
source install/setup.bash && ros2 launch fakelidar lidar_node.launch.py &
cd ..

sleep 2
cd slam_sim
source install/setup.bash && ros2 launch slam slam_node.launch.py &
cd ..

sleep 1
cd hybrid_a_star_ws
source install/setup.bash && ros2 launch hybrid_a_star hybrid_a_star_launch_file.launch.xml &
cd ..

sleep 1
cd bicycle_controller_ign
source install/setup.bash && ros2 launch ign_ros2_control_demos ackermann_drive_example.launch.py &


#cd simulated_robot
#source install/setup.bash && ros2 run hunter_base plot_hunter.py &
#cd ..
# Keep script running
wait
