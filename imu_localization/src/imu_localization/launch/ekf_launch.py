from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('imu_localization'), 'cfg', 'ekf_config.yaml'
    )

    print(f"Config path: {config_path}")
    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[config_path, {"use_sim_time": False, "debug": False}],
        )
    ])



# arguments=["--ros-args", "--log-level", "debug"] debug arguments
