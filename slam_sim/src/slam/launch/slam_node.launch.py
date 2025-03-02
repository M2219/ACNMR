import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    slam_share = get_package_share_directory('slam')
    amcl_params_file = os.path.join(slam_share, 'cfg', 'amcl_params.yaml')

    return LaunchDescription([
        # Lifecycle Manager
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["amcl"]
            }]
        ),

        # AMCL Node
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[amcl_params_file, {
                "use_sim_time": False,
                "initial_pose_x": 0.0,
                "initial_pose_y": 0.0,
                "initial_pose_a": 0.0,
                "base_frame_id": "base_link",
                "global_frame_id": "map",
                "odom_frame_id": "odom",
                "scan_topic": "/scan"
            }]
        )
    ])
