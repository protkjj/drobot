#!/usr/bin/env python3
"""
Drobot Gazebo Simulation Launch File
ROS 2 Jazzy + Gazebo Harmonic

실행 방법:
  ros2 launch drobot_simulation simulation.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 패키지 경로
    pkg_drobot_sim = get_package_share_directory('drobot_simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # URDF 파일 경로
    urdf_file = os.path.join(pkg_drobot_sim, 'urdf', 'drobot.urdf.xacro')

    # World 파일 경로
    world_file = os.path.join(pkg_drobot_sim, 'worlds', 'empty.sdf')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

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

    # 로봇 스폰
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'drobot',
            '-z', '0.1'  # 살짝 위에서 스폰
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge (토픽 연결)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # cmd_vel: ROS -> Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # odom: Gazebo -> ROS
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # joint_states: Gazebo -> ROS
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            # IMU: Gazebo -> ROS
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            # LiDAR: Gazebo -> ROS
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Nodes
        robot_state_publisher,
        gazebo,
        spawn_robot,
        ros_gz_bridge,
    ])
