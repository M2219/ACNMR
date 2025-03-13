from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Get package directory
    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')

    # Path to the world file
    world_file = os.path.join(
        get_package_share_directory('ackermann_inputs'),
        'worlds',
        'simple_world.world'
    )

    # Include the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    return LaunchDescription([
        gazebo_launch
    ])
