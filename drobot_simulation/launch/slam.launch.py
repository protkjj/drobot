#!/usr/bin/env python3
"""
SLAM Launch File for Drobot
slam_toolbox를 사용한 실시간 지도 생성
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_drobot_sim = get_package_share_directory('drobot_simulation')
    slam_params_file = os.path.join(pkg_drobot_sim, 'config', 'slam_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # SLAM Toolbox 노드
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        slam_toolbox,
    ])
