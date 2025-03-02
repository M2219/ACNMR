import os
import time
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('map_creator')
    map_file = os.path.join(package_dir, "maps", "map_slalom.yaml")

    return LaunchDescription([

        # Static TF
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_map",
            arguments=["0", "0", "0", "0", "0", "0", "world", "map"]
        ),

        # Lifecycle Manager to Ensure Activation
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map_server",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["map_server"]
            }],
        ),

        # Delay to Give Lifecycle Manager Time to Activate
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "frame_id": "map",
                "yaml_filename": map_file
            }],
            on_exit=[Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_server",
                output="screen",
                parameters=[{"autostart": True, "node_names": ["map_server"]}]
            )]
        ),

        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", os.path.join(package_dir, "cfg", "rviz_navigation.rviz")],
            output="screen"
        )
    ])
