#!/usr/bin/env python3
"""
Full Navigation Launch File
시뮬레이션 + Localization + Nav2 한번에 실행

사전 조건:
  - 지도 파일 필요 (먼저 SLAM으로 생성)

실행 방법:
  ros2 launch drobot_simulation full_navigation.launch.py map:=~/my_map.yaml
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
    nav2_params_file = os.path.join(pkg_drobot_sim, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_drobot_sim, 'config', 'nav2_view.rviz')

    # 1. Simulation (Gazebo + Robot)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_drobot_sim, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Localization (EKF)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_drobot_sim, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. Nav2 (delayed start to wait for simulation)
    nav2_bringup = TimerAction(
        period=5.0,  # 5초 후 시작
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_yaml_file,
                    'params_file': nav2_params_file,
                }.items()
            )
        ]
    )

    # 4. RViz with Nav2 config (replace default)
    rviz2 = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_nav',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
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

        # Launch sequence
        simulation,
        localization,
        nav2_bringup,
        # rviz2,  # simulation.launch.py에서 이미 RViz 실행하므로 주석처리
    ])
