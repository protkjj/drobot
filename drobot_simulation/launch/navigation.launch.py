#!/usr/bin/env python3
"""
Nav2 Navigation Launch File
자율 주행을 위한 Nav2 스택 실행

사전 조건:
  1. simulation.launch.py 실행 중
  2. 지도 파일 필요 (SLAM으로 생성)

실행 방법:
  ros2 launch drobot_simulation navigation.launch.py map:=/path/to/map.yaml
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 패키지 경로
    pkg_drobot_sim = get_package_share_directory('drobot_simulation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default=os.path.expanduser('~/my_map.yaml'))
    nav2_params_file = LaunchConfiguration('params_file',
        default=os.path.join(pkg_drobot_sim, 'config', 'nav2_params.yaml'))

    # Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': nav2_params_file,
        }.items()
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/my_map.yaml'),
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_drobot_sim, 'config', 'nav2_params.yaml'),
            description='Full path to Nav2 parameters file'
        ),

        # Nav2
        nav2_bringup,
    ])
