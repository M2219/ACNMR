from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthai_oakdpro',
            executable='sync_sanity_checker',
            name='sync_sanity_checker',
            output='screen'
        )
    ])
