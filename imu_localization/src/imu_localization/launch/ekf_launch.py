from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('imu_localization'), 'cfg', 'ekf_config.yaml'
    )

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_localization_node",
            output="screen",
            parameters=[config_path]
        )
    ])
