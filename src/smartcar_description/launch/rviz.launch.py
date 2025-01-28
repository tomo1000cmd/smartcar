from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join

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

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher'
    )

    # Robot State Publisher to load the URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time if true'),
        joint_state_publisher,
        robot_state_publisher,
        rviz
    ])
