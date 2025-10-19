#!/usr/bin/env python3
"""
Complete SmartCar System Launch File
Launches Gazebo, Localization, and Navigation in the correct order
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions as launch_conditions
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Package directories
    pkg_smartcar = get_package_share_directory('smartcar')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')
    enable_localization = LaunchConfiguration('localization')
    enable_navigation = LaunchConfiguration('navigation')
    map_file = LaunchConfiguration('map_file')
    slam = LaunchConfiguration('slam')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([pkg_smartcar, 'maps', 'smalltown.world']),
        description='Full path to the world file to load'
    )
    
    # RViz will be launched by Nav2 launch file
    
    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='true',
        description='Launch localization stack (EKF + wheel odometry)'
    )
    
    declare_navigation_cmd = DeclareLaunchArgument(
        'navigation',
        default_value='true',
        description='Launch Nav2 navigation stack'
    )
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([pkg_smartcar, 'maps', 'smalltown_world.yaml']),
        description='Full path to the map file'
    )
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Whether to run SLAM instead of using pre-existing map'
    )
    
    # ========================
    # 1. Launch Gazebo Simulation
    # ========================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_smartcar, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world_file': world_file,
            'use_sim_time': use_sim_time,
            'x_pose': '0.0',
            'y_pose': '0.0'
        }.items()
    )
    
    # ========================
    # 2. Launch Navigation (after localization)
    # ========================
    navigation_launch = TimerAction(
        period=15.0,  # Wait 15 seconds for localization to fully stabilize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([pkg_smartcar, 'launch', 'nav2.launch.py'])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_file,
                    'slam': slam
                }.items(),
                condition=IfCondition(enable_navigation)
            )
        ]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_localization_cmd)
    ld.add_action(declare_navigation_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_slam_cmd)
    
    # Add launch files and nodes in order
    ld.add_action(gazebo_launch)
    ld.add_action(navigation_launch)
    
    return ld
