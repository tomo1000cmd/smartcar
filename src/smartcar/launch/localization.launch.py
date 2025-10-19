#!/usr/bin/env python3
"""
Localization Launch File for SmartCar
Launches wheel odometry, joint state publisher, and EKF for sensor fusion
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_smartcar = get_package_share_directory('smartcar')
    
    # Configuration files
    ekf_config = os.path.join(pkg_smartcar, 'config', 'ekf.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Wheel Odometry Node
    wheel_odom_node = Node(
        package='smartcar',
        executable='wheel_odom.py',
        name='wheel_odom',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Joint State Publisher (for fixed joints)
    joint_state_publisher_node = Node(
        package='smartcar',
        executable='joint_state_publisher.py',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # EKF Node for Sensor Fusion
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/odometry/filtered', '/odom')
        ]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add nodes
    ld.add_action(wheel_odom_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(ekf_node)
    
    return ld
