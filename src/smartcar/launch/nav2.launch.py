#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('smartcar')
    nav2_share = get_package_share_directory('nav2_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    
    # Declare launch arguments
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Whether to run SLAM (Simultaneous Localization and Mapping)'
    )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'maps', 'smalltown_world.yaml'),
        description='Full path to map yaml file to load'
    )
    
    # SmartCar localization stack (wheel odom + EKF)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # NOTE: robot_state_publisher is already launched by gazebo.launch.py
    # Do not duplicate it here to avoid conflicts

    # Joint State Publisher disabled - Gazebo publishes joint states via custom plugin
    # Uncomment below if running without Gazebo simulation
    # joint_state_publisher_gui = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher_gui',
    #     parameters=[{'robot_description': robot_description}],
    #     condition=IfCondition(use_gui)
    # )
    
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robot_description}]
    # )

    # NOTE: Transform hierarchy:
    # - odom -> base_footprint comes from wheel_odom + EKF (localization.launch.py)
    # - map -> odom comes from AMCL (no static transform needed)

    # SLAM Toolbox (only when slam:=true)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'debug_logging': False,
                'throttle_scans': 1,
                'transform_publish_period': 0.02,
                'map_update_interval': 5.0,
                'resolution': 0.05,
                'max_laser_range': 12.0,
                'minimum_time_interval': 0.5,
                'transform_timeout': 0.2,
                'tf_buffer_duration': 30.0,
                'stack_size_to_use': 40000000,
                'enable_interactive_mode': False,
                'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
                'ceres_preconditioner': 'SCHUR_JACOBI',
                'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
                'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
                'ceres_loss_function': 'None'
            }
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        condition=IfCondition(slam)
    )
    
    # Map server (only when slam:=false, i.e., using pre-existing map)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'yaml_filename': map_yaml_file,
            'use_sim_time': use_sim_time,
            'frame_id': 'map'
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        condition=UnlessCondition(slam)
    )
    
    # AMCL (only when slam:=false, i.e., using pre-existing map for localization)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[os.path.join(pkg_share, 'config', 'nav2_params.yaml')],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        condition=UnlessCondition(slam)
    )
    
    # Lifecycle manager for localization (when not using SLAM)
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
             'autostart': True,
             'node_names': ['map_server', 'amcl']}
        ],
        condition=UnlessCondition(slam)
    )
    
    # SLAM Toolbox manages its own lifecycle, so no separate lifecycle manager needed
    
    # Navigation stack nodes
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[os.path.join(pkg_share, 'config', 'nav2_params.yaml')],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )
    
    # Collision Monitor
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[os.path.join(pkg_share, 'config', 'nav2_params.yaml')],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'nav2_params.yaml')],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'nav2_params.yaml')],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'nav2_params.yaml')],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'nav2_params.yaml')],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'nav2_params.yaml')],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # Lifecycle manager for localization (when not using SLAM)
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
             'autostart': True,
             'node_names': ['map_server', 'amcl']}
        ],
        condition=UnlessCondition(slam)
    )

    # Lifecycle manager for navigation
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
             'autostart': True,
             'node_names': ['controller_server', 'smoother_server', 'planner_server', 
                           'behavior_server', 'bt_navigator', 'velocity_smoother']}
        ]
    )

    # Teleop keyboard for manual control (disabled when navigation is active to avoid conflicts)
    # teleop_node = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop_twist_keyboard',
    #     prefix='xterm -e',
    #     output='screen'
    # )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'smartcar.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',  # Default to false for standalone launch
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Launch joint state publisher gui'),
        DeclareLaunchArgument(
            'slam',
            default_value='false',
            description='Whether to run SLAM (Simultaneous Localization and Mapping)'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'maps', 'smalltown_world.yaml'),
            description='Full path to map yaml file to load'),
        # robot_state_publisher is launched by gazebo.launch.py - do not duplicate
        # joint_state_publisher_gui,  # Disabled - conflicts with Gazebo
        # joint_state_publisher,      # Disabled - conflicts with Gazebo  
        localization_launch,
        slam_toolbox,  # SLAM Toolbox (conditional)
        map_server,    # Map server (conditional) 
        amcl,          # AMCL (conditional)
        lifecycle_manager_localization,
        controller_server,
        collision_monitor,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager_navigation,
        # teleop_node,
        rviz_node
    ])