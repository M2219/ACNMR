import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="fakelidar",
            executable="fakelidar.py",
            name="fake_lidar",
            output="screen",
            parameters=[
                {"lidar_range": 5.0},
                {"angle_min": -1.57},
                {"angle_max": 1.57},
                {"angle_increment": 0.01}
            ]
        )
    ])
