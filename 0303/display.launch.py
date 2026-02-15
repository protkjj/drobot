"""
DROBOT RViz Display Launch
- robot_state_publisher (xacro → URDF → /robot_description)
- joint_state_publisher_gui (슬라이더로 joint 테스트)
- rviz2 (시각화)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory('drobot_description')

    # xacro → URDF 변환
    robot_description = Command([
        'xacro ', os.path.join(pkg_path, 'urdf', 'drobot.urdf.xacro'),
        ' use_sim:=false',
        ' use_ros2_control:=false'
    ])

    # RViz config (있으면 사용)
    rviz_config = os.path.join(pkg_path, 'config', 'display.rviz')

    return LaunchDescription([

        # === robot_state_publisher ===
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # === joint_state_publisher_gui (슬라이더) ===
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),

        # === RViz2 ===
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            output='screen'
        ),
    ])
