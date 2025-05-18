import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    depthai_node = Node(
        package='kitti_publisher',
        executable='kitti_publisher_cuda_node',
        name='kitti_publisher_cuda_node',
        output='screen',
        parameters=[{'kitti_path': '/path/to/sequence'}]
    )

    return LaunchDescription([
        depthai_node,
    ])
