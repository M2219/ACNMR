import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    depthai_node = Node(
        package='euroc_publisher',
        executable='euroc_publisher_cuda_node',
        name='euroc_publisher_cuda_node',
        output='screen',
        parameters=[{'bag_path': '/path/to/MH_01_easy/MH_01_easy.db3'}]
    )

    return LaunchDescription([
        depthai_node,
    ])
