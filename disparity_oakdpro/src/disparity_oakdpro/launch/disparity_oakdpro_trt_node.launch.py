import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_dir = os.path.join(
        get_package_share_directory('disparity_oakdpro'),
        'config'
    )
    model_path = os.path.join(config_dir, 'model_ghustereo8_nce.plan')

    disparity_node_trt = Node(
        package='disparity_oakdpro',
        executable='disparity_oakdpro_trt_node',
        name='disparity_oakdpro_trt_node',
        output='screen',
        parameters=[{'model_path': model_path}]
    )

    return LaunchDescription([
        disparity_node_trt,
    ])
