import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, SetEnvironmentVariable,
    ExecuteProcess, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('drobot_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'drobot.urdf.xacro')

    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_dir, '..')
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=true use_ros2_control:=false']),
        value_type=str
    )

    # Start Gazebo PAUSED (no -r flag)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
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

    # After 3s: unpause Gazebo
    unpause = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['gz', 'service', '-s', '/world/empty/control',
                     '--reqtype', 'gz.msgs.WorldControl',
                     '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '2000',
                     '--req', 'pause: false'],
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
