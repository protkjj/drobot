#!/usr/bin/env python3
"""
메인 런치 파일 - 전체 시스템 실행
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory('robot_controller')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        # ========== Arguments ==========
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # ========== Mode Manager ==========
        Node(
            package='robot_controller',
            executable='mode_manager_node',
            name='mode_manager',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ]
        ),

        # ========== Wheel Controller ==========
        Node(
            package='robot_controller',
            executable='wheel_controller_node',
            name='wheel_controller',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ]
        ),

        # ========== PX4 Interface ==========
        Node(
            package='robot_controller',
            executable='px4_interface_node',
            name='px4_interface',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ]
        ),
    ])
