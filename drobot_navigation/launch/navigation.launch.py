#!/usr/bin/env python3
"""
Drobot Navigation VER1 Launch File

Goal-based autonomous navigation with rule engine.
Standalone launch - drobot_simulation 패키지 없이 독립 실행 가능.

Usage:
  ros2 launch drobot_navigation navigation.launch.py
  ros2 launch drobot_navigation navigation.launch.py world:=f1_circuit
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from ros_gz_bridge.actions import RosGzBridge


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
    desc_pkg = get_package_share_directory('drobot_description')
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # Get launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = context.launch_configurations.get('world', 'empty')

    # Get spawn position for this world
    default_x, default_y = WORLD_SPAWN_POSITIONS.get(world, (0.0, 0.0))
    spawn_x = str(default_x)
    spawn_y = str(default_y)

    # URDF (from drobot_description)
    urdf_file = os.path.join(desc_pkg, 'urdf', 'drobot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # World file (from drobot_description)
    world_file = os.path.join(desc_pkg, 'worlds', f'{world}.sdf')
    if not os.path.exists(world_file):
        world_file = os.path.join(desc_pkg, 'worlds', 'generated', f'{world}.sdf')
    if not os.path.exists(world_file):
        print(f"[WARNING] World file not found: {world}.sdf")
        print(f"  Searched: worlds/ and worlds/generated/")
        world_file = os.path.join(desc_pkg, 'worlds', 'empty.sdf')

    # Config files (all from nav_pkg)
    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    bt_xml = os.path.join(nav_pkg, 'config', 'navigate_with_replanning.xml')
    slam_params = os.path.join(nav_pkg, 'config', 'slam_params.yaml')
    ekf_params = os.path.join(nav_pkg, 'config', 'ekf.yaml')
    bridge_config = os.path.join(nav_pkg, 'config', 'ros_gz_bridge.yaml')
    rviz_config = os.path.join(nav_pkg, 'config', 'display.rviz')

    # ========== Simulation Nodes ==========

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

    # Gazebo Sim (Harmonic)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Robot Spawn
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'drobot',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', '0.1'
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config,
    )

    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ========== Localization Nodes ==========

    # EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': True}],
    )

    # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )

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

    # ========== Navigation Nodes ==========

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {
            'default_nav_to_pose_bt_xml': bt_xml,
        }],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
    )

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
        # Simulation
        robot_state_publisher,
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        rviz2,
        # Localization
        ekf_node,
        slam_node,
        slam_lifecycle,
        # Navigation
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
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='World name (empty, warehouse, f1_circuit, office_maze, param_test)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
