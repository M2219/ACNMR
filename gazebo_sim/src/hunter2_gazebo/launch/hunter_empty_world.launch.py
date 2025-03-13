import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions
from ament_index_python.packages import get_package_share_directory
import launch
import launch.launch_description_sources
import launch.substitutions
import launch_ros
import launch_ros.substitutions

def generate_launch_description():

  # Constants for paths to different files and folders
  robot_name_in_model = 'hunter2'

  # Pose where we want to spawn the robot
  spawn_x_val = '0.0'
  spawn_y_val = '0.0'
  spawn_z_val = '0.0'
  spawn_yaw_val = '0.00'

  # Set the path to different files and folders.
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
  package_share_directory = get_package_share_directory('hunter2_gazebo')

  urdf_file_path = os.path.join(package_share_directory, 'urdf', 'hunter2_base_gazebo.xacro')
  world_file_path = os.path.join(package_share_directory, 'world', 'neighborhood.world')

  default_urdf_model_path = urdf_file_path
  world_path = world_file_path

  ackermann_inputs_pkg = get_package_share_directory('ackermann_inputs')
  # Launch configuration variables specific to simulation
  use_sim_time = LaunchConfiguration('use_sim_time', default='True')
  gui = LaunchConfiguration('gui')
  headless = LaunchConfiguration('headless')
  namespace = LaunchConfiguration('namespace')
  urdf_model = LaunchConfiguration('urdf_model')
  use_namespace = LaunchConfiguration('use_namespace')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')

  # Declare the launch arguments
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

  declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
    name='gui',
    default_value='True',
    description='Flag to enable joint_state_publisher_gui')

  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')

  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='False',
    description='Whether to apply a namespace to the navigation stack')


  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')

  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model',
    default_value=default_urdf_model_path,
    description='Absolute path to robot urdf file')

  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')


  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
  #  )

  # Publish the joint states of the robot
  start_joint_state_publisher_cmd = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    condition=UnlessCondition(gui),
    parameters=[{'use_sim_time': use_sim_time}])

  start_joint_state_publisher_gui_node = Node(
    condition=IfCondition(gui),
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
    parameters=[{'use_sim_time': use_sim_time}])

  # start_dummy_sensors=Node(
  #   package='dummy_sensors',
  #   node_executable='dummy_joint_states',
  #   output='screen')


  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    # parameters=[{'robot_description': Command(['xacro ', urdf_model]),'use_sim_time': use_sim_time}]
    parameters=[{'robot_description': launch_ros.descriptions.ParameterValue( launch.substitutions.Command([
      'xacro ', urdf_file_path]), value_type=str)  }]
    )

  # Publish the joint states of the robot
  start_joint_state_publisher_cmd = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    condition=UnlessCondition(gui),
    parameters=[{'use_sim_time': use_sim_time}])

  start_joint_state_publisher_gui_node = Node(
    condition=IfCondition(gui),
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
    parameters=[{'use_sim_time': use_sim_time}])

  # start_dummy_sensors=Node(
  #   package='dummy_sensors',
  #   node_executable='dummy_joint_states',
  #   output='screen')

  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world, 'use_sim_time': 'True'}.items())

  # Start Gazebo client
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

  # Launch the robot
  spawn_entity_cmd = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-entity', robot_name_in_model,
                '-topic', 'robot_description',
                    '-x', spawn_x_val,
                    '-y', spawn_y_val,
                    '-z', spawn_z_val,
                    '-Y', spawn_yaw_val],
                    output='screen')


 # Spawn the custom Ackermann controller plugin
 # controller = Node(
 #       package="controller_manager",
 #       executable="spawner",
 #       arguments=["gazebo_ros_ackermann_drive"],
 #   )


 # controller = Node(
 #   package='controller_manager',  # Controller manager package
 #   executable='spawner',           # spawner executable that loads controllers
 #   arguments=['ackermann_inputs'],
 #   )

  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_joint_state_publisher_cmd)
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)

  #ld.add_action(controller)

  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(spawn_entity_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_joint_state_publisher_gui_node)
  ld.add_action(start_joint_state_publisher_cmd)
  # ld.add_action(start_dummy_sensors)
  return ld
