from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Locate the path to your URDF/Xacro file
    xacro_file = FindPackageShare('smartcar').find('smartcar_description') + '/model/smartcar.urdf.xacro'

 # Locate the path to your URDF/Xacro file
    world_file = FindPackageShare('smart_car').find('smartcar_bringup') + '/worlds/smalltown.world'

    # Convert the Xacro file to URDF
    robot_description = Command(['xacro ', xacro_file])

    # Launch description
    return LaunchDescription([
        # Launch robot_state_publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Launch joint_state_publisher_gui node for manual joint adjustments
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
        
            # Include the Gazebo launch description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={'world': world_file}.items()
        ),

        # Spawn the robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'smartcar'
            ],
            output='screen'
        ),
    ])

