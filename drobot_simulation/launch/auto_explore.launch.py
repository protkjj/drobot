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
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_drobot_sim = get_package_share_directory('drobot_simulation')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='full_world')

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
        }.items()
    )

    # 2. Localization (EKF)
    localization = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_drobot_sim, 'launch', 'localization.launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # 3. SLAM (커스텀 파라미터 사용 + lifecycle 관리)
    slam_params = os.path.join(pkg_drobot_sim, 'config', 'slam_params.yaml')
    slam = TimerAction(
        period=5.0,
        actions=[
            GroupAction([
                # SLAM Toolbox Node
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[slam_params, {'use_sim_time': use_sim_time}],
                ),
                # Lifecycle Manager for SLAM
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_slam',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': True,
                        'node_names': ['slam_toolbox']
                    }],
                ),
            ])
        ]
    )

    # 4. Nav2 (SLAM mode - custom nodes without docking_server)
    nav2_nodes = TimerAction(
        period=8.0,
        actions=[
            GroupAction([
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

    # 5. Auto Explorer (wait for everything to start)
    auto_explorer = TimerAction(
        period=18.0,
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

        simulation,
        localization,
        slam,
        nav2_nodes,
        auto_explorer,
    ])
