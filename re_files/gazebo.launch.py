"""
DROBOT Gazebo Simulation Launch (Gz Sim Harmonic)
- gz sim (Gazebo Harmonic)
- robot_state_publisher
- spawn_entity (로봇 스폰)
- bridge (Gz ↔ ROS2 토픽 브릿지)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory('drobot_description')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_path, 'worlds', 'empty.world'),
        description='Gazebo world file'
    )

    # xacro → URDF 변환
    robot_description = Command([
        'xacro ', os.path.join(pkg_path, 'urdf', 'drobot.urdf.xacro'),
        ' use_sim:=true',
        ' use_ros2_control:=true'
    ])

    # === robot_state_publisher ===
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # === Gz Sim (Harmonic) ===
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r',
            LaunchConfiguration('world'),
        ],
        output='screen'
    )

    # === Spawn Robot ===
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'drobot',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',  # 약간 위에서 스폰
        ],
        output='screen'
    )

    # === Gz ↔ ROS2 Bridge ===
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # cmd_vel (ROS2 → Gz)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # odom (Gz → ROS2)
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # LiDAR (Gz → ROS2)
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            # Camera (Gz → ROS2)
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            # IMU (Gz → ROS2)
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            # Joint states (Gz → ROS2)
            '/world/default/model/drobot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
            # Arm commands (ROS2 → Gz)
            '/drobot/arm_front_cmd@std_msgs/msg/Float64@gz.msgs.Double',
            '/drobot/arm_rear_cmd@std_msgs/msg/Float64@gz.msgs.Double',
            '/drobot/tilt_cmd@std_msgs/msg/Float64@gz.msgs.Double',
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
