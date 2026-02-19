from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    linear_speed = LaunchConfiguration('linear_speed')
    angular_speed = LaunchConfiguration('angular_speed')

    return LaunchDescription([
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.35',
            description='Linear speed for keyboard teleop (m/s)',
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.9',
            description='Angular speed for keyboard teleop (rad/s)',
        ),
        Node(
            package='drobot_controller',
            executable='keyboard_teleop',
            name='keyboard_teleop',
            output='screen',
            parameters=[{
                'linear_speed': linear_speed,
                'angular_speed': angular_speed,
            }],
        ),
    ])
