#!/usr/bin/env python3
"""
시뮬레이션 런치 파일 - Gazebo + PX4 SITL 환경용
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory('robot_controller')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([
        # ========== Arguments ==========
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # ========== MicroXRCE-DDS Agent ==========
        # PX4 SITL과 통신을 위한 DDS Agent
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen',
            shell=False
        ),

        # ========== Mode Manager ==========
        Node(
            package='robot_controller',
            executable='mode_manager_node',
            name='mode_manager',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': True}
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
                {'use_sim_time': True}
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
                {'use_sim_time': True}
            ]
        ),
    ])
