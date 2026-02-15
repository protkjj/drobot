"""
DROBOT Gazebo Simulation Launch (Gz Sim Harmonic)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory('drobot_description')

    gz_resource_path = os.path.join(pkg_path, '..')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + gz_resource_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_path, 'worlds', 'empty.sdf'),
        description='Gazebo world file'
    )

    robot_description = Command([
        'xacro ', os.path.join(pkg_path, 'urdf', 'drobot.urdf.xacro'),
        ' use_sim:=true',
        ' use_ros2_control:=true'
    ])

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
        output='screen'
    )

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', LaunchConfiguration('world')],
        output='screen'
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'drobot',
            '-topic', '/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.3',
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/world/empty/model/drobot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/drobot/arm_cmd@std_msgs/msg/Float64@gz.msgs.Double',
        ],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        robot_state_pub,
        gz_sim,
        spawn_entity,
        bridge,
    ])
