import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the package and map file path
    package_dir = FindPackageShare('map_creator').find('map_creator')
    map_file = os.path.join(package_dir, "maps", "map_slalom.yaml")

    # Declare 'use_sim_time' argument to control whether simulation time is used
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true',
                                             description='Use simulation time if true')

    return LaunchDescription([

        # Declare the use of simulation time (this should be passed as an argument)
        use_sim_time_arg,

        # Static TF
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_map",
            arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Lifecycle Manager to Ensure Activation
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map_server",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["map_server"],
                'use_sim_time': LaunchConfiguration('use_sim_time')  # Ensure use_sim_time is set
            }],
        ),

        # Delay to Give Lifecycle Manager Time to Activate
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration('use_sim_time'),
                "frame_id": "map",
                "yaml_filename": map_file
            }],
            on_exit=[Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_server",
                output="screen",
                parameters=[{"autostart": True, "node_names": ["map_server"]}],
                remappings=[('use_sim_time', LaunchConfiguration('use_sim_time'))]
            )]
        ),
    ])
