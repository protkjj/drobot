#!/usr/bin/env python3
"""
Drobot Navigation VER1 Launch File

Goal-based autonomous navigation with rule engine.
Usage:
  ros2 launch drobot_navigation navigation.launch.py
  ros2 launch drobot_navigation navigation.launch.py world:=f1_circuit
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# World-specific spawn positions
WORLD_SPAWN_POSITIONS = {
    # 기본 월드
    'empty': (0.0, 0.0),
    'full_world': (1.0, 1.0),
    'simple_room': (0.0, 0.0),
    'warehouse': (0.0, -3.0),
    'f1_circuit': (0.0, 3.5),
    'office_maze': (-4.0, -3.0),
    # 생성된 월드 (worlds/generated/)
    'room_generated': (0.0, 0.0),
    'maze_generated': (-8.0, -8.0),
    'road_test': (0.0, -3.0),
    'road_simple': (0.0, -1.0),
    'param_test': (0.0, 0.0),
}


def launch_setup(context):
    """Setup launch with context for spawn positions."""
    # Package directories
    nav_pkg = get_package_share_directory('drobot_navigation')
    sim_pkg = get_package_share_directory('drobot_simulation')

    # Get world name
    world = context.launch_configurations.get('world', 'empty')

    # Get spawn position for this world
    default_x, default_y = WORLD_SPAWN_POSITIONS.get(world, (0.0, 0.0))
    spawn_x = str(default_x)
    spawn_y = str(default_y)

    # Config files
    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    bt_xml = os.path.join(nav_pkg, 'config', 'navigate_with_replanning.xml')
    slam_params = os.path.join(sim_pkg, 'config', 'slam_params.yaml')
    ekf_params = os.path.join(sim_pkg, 'config', 'ekf.yaml')

    # Include simulation launch with spawn position
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'world': world,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
        }.items()
    )

    # EKF for localization
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': True}],
    )

    # SLAM Toolbox (async mode for real-time mapping)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )

    # Nav2 Lifecycle Manager for SLAM
    # bond_timeout: 0.0 disables heartbeat monitoring (recommended for simulation)
    slam_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['slam_toolbox']
        }],
    )

    # Nav2 Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
    )

    # Nav2 Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    # Nav2 Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    )

    # Nav2 BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {
            'default_nav_to_pose_bt_xml': bt_xml,
        }],
    )

    # Velocity Smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
    )

    # Nav2 Lifecycle Manager for Navigation
    # bond_timeout: 0.0 disables heartbeat monitoring (recommended for simulation)
    nav_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'velocity_smoother',
            ]
        }],
    )

    # Goal Navigator (VER1 main node)
    goal_navigator = Node(
        package='drobot_navigation',
        executable='goal_navigator.py',
        name='goal_navigator',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return [
        simulation_launch,
        ekf_node,
        slam_node,
        slam_lifecycle,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        nav_lifecycle,
        goal_navigator,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='World name (empty, warehouse, f1_circuit, office_maze, param_test)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
