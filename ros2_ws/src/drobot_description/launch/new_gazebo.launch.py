import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('drobot_description')
    xacro_file = os.path.join(pkg_dir, 'urdf_sub', 'drobotv0.urdf.xacro')

    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_dir, '..')
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': 'empty.sdf'}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'drobot',
            '-z', '0.15',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen',
    )

    unpause = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz',
                    'service',
                    '-s',
                    '/world/empty/control',
                    '--reqtype',
                    'gz.msgs.WorldControl',
                    '--reptype',
                    'gz.msgs.Boolean',
                    '--timeout',
                    '2000',
                    '--req',
                    'pause: false',
                ],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        gz_resource_path,
        gz_sim,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        spawn,
        bridge,
        unpause,
    ])
