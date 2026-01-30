#!/usr/bin/env python3
"""
텔레오퍼레이션 런치 파일 - 키보드 조작용
RC카 모드에서 키보드로 조종할 때 사용
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
    use_keyboard = LaunchConfiguration('use_keyboard', default='true')

    return LaunchDescription([
        # ========== Arguments ==========
        DeclareLaunchArgument(
            'use_keyboard',
            default_value='true',
            description='Use custom keyboard teleop node'
        ),

        # ========== Wheel Controller ==========
        Node(
            package='robot_controller',
            executable='wheel_controller_node',
            name='wheel_controller',
            output='screen',
            parameters=[params_file]
        ),

        # ========== Teleop Keyboard ==========
        Node(
            package='robot_controller',
            executable='teleop_keyboard_node',
            name='teleop_keyboard',
            output='screen',
            parameters=[params_file],
            prefix='xterm -e',  # 새 터미널에서 실행 (키보드 입력용)
        ),
    ])
