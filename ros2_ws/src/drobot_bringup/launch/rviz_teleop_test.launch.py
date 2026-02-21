import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg = get_package_share_directory('drobot_description')
    xacro_file = os.path.join(description_pkg, 'urdf_sub', 'drobotv0.urdf.xacro')
    rviz_config = os.path.join(description_pkg, 'config', 'display.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
        Node(
            package='drobot_bringup',
            executable='fake_diff_odom',
            name='fake_diff_odom',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
                'rate_hz': 30.0,
                'cmd_timeout_sec': 0.5,
            }],
        ),
    ])
