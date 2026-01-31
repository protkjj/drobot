#!/usr/bin/env python3
"""
Drobot Gazebo Simulation Launch File
ROS 2 Jazzy + Gazebo Harmonic

실행 방법:
  ros2 launch drobot_simulation simulation.launch.py
  ros2 launch drobot_simulation simulation.launch.py world:=f1_circuit
  ros2 launch drobot_simulation simulation.launch.py world:=warehouse

사용 가능한 월드:
  - empty (기본, 장애물 있는 방)
  - f1_circuit (F1 서킷)
  - simple_room (거실)
  - warehouse (창고)
  - office_maze (사무실)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ros_gz_bridge.actions import RosGzBridge


def launch_setup(context, *args, **kwargs):
    # 패키지 경로
    pkg_drobot_sim = get_package_share_directory('drobot_simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # URDF 파일 경로
    urdf_file = os.path.join(pkg_drobot_sim, 'urdf', 'drobot.urdf.xacro')

    # Launch arguments 값 가져오기
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world').perform(context)

    # World 파일 경로
    world_file = os.path.join(pkg_drobot_sim, 'worlds', f'{world_name}.sdf')

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
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Gazebo Sim (Harmonic) 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # 로봇 스폰 (거실 빈 공간에서 시작)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'drobot',
            '-x', '-1.0',
            '-y', '-2.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge 설정 파일
    bridge_config = os.path.join(pkg_drobot_sim, 'config', 'ros_gz_bridge.yaml')

    # ROS-Gazebo Bridge (토픽 연결) - RosGzBridge 액션 사용
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config,
    )

    # RViz2
    rviz_config = os.path.join(pkg_drobot_sim, 'config', 'display.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return [
        robot_state_publisher,
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        rviz2,
    ]


def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='World name (empty, f1_circuit, simple_room, warehouse, office_maze)'
        ),

        # OpaqueFunction으로 런타임에 월드 경로 결정
        OpaqueFunction(function=launch_setup),
    ])
