import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheel_odometry_gazebo',             # Package name
            executable='publish_wheel_odometry_gazebo',  # Node executable
            name='wheel_odometry_gazebo_node',           # Name for the node
            output='screen',                     # Output the logs to the screen
            parameters=[{'use_sim_time': True}],  # Example parameter (sim time)
        ),
    ])
