
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from os.path import join
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to packages
    description_package = get_package_share_directory('smartcar_description')
    bringup_package = get_package_share_directory('smartcar_bringup')
    map_server_package = get_package_share_directory('smartcar_map')
    # URDF and world file paths
    urdf_file = join(description_package, 'model', 'smartcar.urdf.xacro')
    world_file = join(bringup_package, 'worlds', 'smalltown.world')
    map_file = join(map_server_package, 'worlds', 'smalltown_world.yaml')
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
 # Convert the Xacro file to URDF
    robot_description = Command(['xacro ', urdf_file])
 
    # Launch Gazebo with the specified world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
     # Bridge clock 
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )



    # Robot State Publisher to load the URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )


    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )
     # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_smartcar',
        arguments=[
            '-entity', 'smartcar',                # Name of the robot
            '-topic', '/robot_description',      # Pass the robot description parameter
        ],
        output='screen'
    )
    teleop_node = Node(
    package='teleop_twist_keyboard',
    executable='teleop_twist_keyboard',
    name='teleop_twist_keyboard',
    output='screen',
    remappings=[
        ('/cmd_vel', '/robot/cmd_vel')  # Adjust topic if necessary
    ]
    ) 

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time if true'),
        gazebo,
        gz_bridge_node,
        map_server,
        robot_state_publisher,
        spawn_entity,
        teleop_node,
       
    ])