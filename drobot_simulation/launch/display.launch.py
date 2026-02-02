#!/usr/bin/env python3
"""
Drobot RViz Display Launch File
URDF 모델 시각화 (Gazebo 없이)

실행 방법:
  ros2 launch drobot_simulation display.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 패키지 경로
    pkg_drobot_desc = get_package_share_directory('drobot_description')  # URDF, meshes, worlds
    pkg_drobot_sim = get_package_share_directory('drobot_simulation')    # config, launch

    # URDF 파일 경로 (drobot_description)
    urdf_file = os.path.join(pkg_drobot_desc, 'urdf', 'drobot.urdf.xacro')

    # RViz 설정 파일 경로 (drobot_simulation)
    rviz_config = os.path.join(pkg_drobot_sim, 'config', 'display.rviz')

    # Launch arguments
    use_gui = LaunchConfiguration('use_gui', default='true')

    # URDF를 xacro로 처리
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # ========== NODES ==========

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    # Joint State Publisher GUI (슬라이더로 조인트 제어)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=None  # use_gui 조건 추가 가능
    )

    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use joint state publisher GUI'
        ),

        # Nodes
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])
