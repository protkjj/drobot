from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')

    # Simple boolean flags derived from mode (string compare in Python launch is annoying)
    # We'll expose them as explicit args for now.
    use_controller = LaunchConfiguration('use_controller')
    use_nav2_2d = LaunchConfiguration('use_nav2_2d')
    use_nav2_3d = LaunchConfiguration('use_nav2_3d')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='',
                             description='Absolute path to a world file (.sdf/.world)'),
        DeclareLaunchArgument('mode', default_value='controller',
                             description='controller | nav2_2d | nav2_3d'),

        # UI will set these explicitly so we avoid string comparisons here
        DeclareLaunchArgument('use_controller', default_value='true'),
        DeclareLaunchArgument('use_nav2_2d', default_value='false'),
        DeclareLaunchArgument('use_nav2_3d', default_value='false'),

        # --- SIMULATION (replace this Node with your actual sim launch if needed) ---
        # If you already have drobot_simulator sim.launch.py, you can swap to IncludeLaunchDescription later.
        # For now, we just print a warning if world is empty by relying on UI to pass it.
        Node(
            package='drobot_simulator',
            executable='sim',  # <-- if you don't have an executable named "sim", we will adjust after you show your sim.launch.py
            name='drobot_sim',
            output='screen',
            parameters=[{'world': world}],
        ),

        # --- CONTROLLER MODE (teleop keyboard placeholder) ---
        GroupAction(
            condition=IfCondition(use_controller),
            actions=[
                Node(
                    package='teleop_twist_keyboard',
                    executable='teleop_twist_keyboard',
                    name='teleop_keyboard',
                    output='screen',
                    prefix='xterm -e'  # optional; remove if you don't want new terminal
                ),
            ]
        ),

        # --- NAV2 2D MODE (placeholder) ---
        GroupAction(
            condition=IfCondition(use_nav2_2d),
            actions=[
                # Put your slam_toolbox/nav2 launch includes here later
            ]
        ),

        # --- NAV2 3D / DRONE MODE (placeholder) ---
        GroupAction(
            condition=IfCondition(use_nav2_3d),
            actions=[
                # Put your drone stack here later (PX4, 3D SLAM, etc.)
            ]
        ),
    ])
