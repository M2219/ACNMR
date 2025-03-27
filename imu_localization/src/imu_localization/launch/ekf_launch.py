from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory
import math


def generate_launch_description():

    distance = 5.8
    lateral_error = 2
    yaw_correction = 0 * math.atan2(lateral_error, distance)


    config_path = os.path.join(
        get_package_share_directory('imu_localization'), 'cfg', 'ekf_config.yaml'
    )

    print(f"Config path: {config_path}")

    return LaunchDescription([
        # EKF Node
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[config_path, {"use_sim_time": False, "debug": False}],
        ),

        # Static transform from base_link to id01_base_link
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            arguments=["0", "0", "0", "0.0", "0.0", str(yaw_correction), "base_link", "id01_base_link"],
            output="screen"
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="id01_to_sensor",
            arguments=["0", "0", "0", "0.0", "0.0", "0.0", "id01_base_link", "id01_sensor_link"],
            output="screen"
        )

    ])
