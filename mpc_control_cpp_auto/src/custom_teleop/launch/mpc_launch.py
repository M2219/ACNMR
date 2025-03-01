import launch
import launch_ros.actions
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="custom_teleop",
            executable="mpc_control_node",
            name="mpc_control_node",
            output="screen",
            parameters=[{"repeat_rate": ParameterValue(100.0)}],  # Set default parameter
        ),
    ])
