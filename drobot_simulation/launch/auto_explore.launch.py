#!/usr/bin/env python3
"""
Auto Exploration Launch File
SLAM + Nav2 + Auto Explorer 한번에 실행

로봇이 자동으로 돌아다니면서 지도를 완성합니다.

실행 방법:
  ros2 launch drobot_simulation auto_explore.launch.py
  ros2 launch drobot_simulation auto_explore.launch.py world:=full_world
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_drobot_sim = get_package_share_directory('drobot_simulation')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='full_world')
    spawn_x = LaunchConfiguration('spawn_x', default='0.0')
    spawn_y = LaunchConfiguration('spawn_y', default='0.0')

    # Nav2 params
    nav2_params = os.path.join(pkg_drobot_sim, 'config', 'nav2_params.yaml')

    # Rewritten params with use_sim_time
    configured_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    # 1. Simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_drobot_sim, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
        }.items()
    )

    # 2. Localization (EKF) - Gazebo 시작 후 대기
    localization = TimerAction(
        period=5.0,  # Gazebo /clock 안정화 대기
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_drobot_sim, 'launch', 'localization.launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # 3. SLAM + Nav2 (lifecycle으로 통합 관리)
    slam_params = os.path.join(pkg_drobot_sim, 'config', 'slam_params.yaml')

    slam_and_nav2 = TimerAction(
        period=8.0,  # Gazebo + EKF 안정화 후 시작
        actions=[
            GroupAction([
                # SLAM Toolbox (lifecycle 노드로 관리)
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[slam_params, {'use_sim_time': use_sim_time}],
                ),
                # Lifecycle Manager for SLAM (먼저 SLAM 활성화)
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_slam',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': True,
                        'node_names': ['slam_toolbox'],
                        'bond_timeout': 0.0,  # SLAM은 bond 비활성화 (호환성)
                    }],
                ),
                # Controller Server
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[configured_params],
                ),
                # Planner Server
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[configured_params],
                ),
                # Behavior Server
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[configured_params],
                ),
                # BT Navigator
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[configured_params],
                ),
                # Velocity Smoother
                Node(
                    package='nav2_velocity_smoother',
                    executable='velocity_smoother',
                    name='velocity_smoother',
                    output='screen',
                    parameters=[configured_params],
                    remappings=[
                        ('cmd_vel', 'cmd_vel_nav'),
                        ('cmd_vel_smoothed', 'cmd_vel')
                    ],
                ),
                # Lifecycle Manager for Nav2
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': True,
                        'node_names': [
                            'controller_server',
                            'planner_server',
                            'behavior_server',
                            'bt_navigator',
                            'velocity_smoother',
                        ]
                    }],
                ),
            ])
        ]
    )

    # 4. Auto Explorer (SLAM + Nav2 lifecycle 활성화 후 시작)
    auto_explorer = TimerAction(
        period=18.0,  # SLAM(8s) + lifecycle 활성화(~10s) 대기
        actions=[
            Node(
                package='drobot_simulation',
                executable='auto_explorer.py',
                name='auto_explorer',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'min_frontier_size': 5,
                    'exploration_timeout': 60.0,
                }]
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='full_world'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),

        simulation,
        localization,
        slam_and_nav2,
        auto_explorer,
    ])
