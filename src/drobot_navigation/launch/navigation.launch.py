import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. 경로 설정
    desc_pkg = get_package_share_directory('drobot_description')
    
    # [설정 1] 인자 정의: 기본값을 'none'으로 설정
    world_conf = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='none',
        description='World file name'
    )

    # [설정 2] 경로 로직 수정
    # 'none'이면 가제보 내장 'empty.sdf'를, 아니면 내 패키지의 worlds/파일명 경로를 생성
    world_path = PythonExpression([
        "'empty.sdf' if '", world_conf, "' == 'none' else '", 
        os.path.join(desc_pkg, 'worlds'), "/' + '", world_conf, "'"
    ])

    # [설정 3] 메쉬 경로 설정 (로봇이 투명하게 보이는 문제 방지)
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(desc_pkg, '..')]
    )

    # 2. URDF/Xacro 처리
    xacro_file = os.path.join(desc_pkg, 'urdf', 'drobot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # 3. Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r ', world_path]}.items(),
    )

    # 4. RSP
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config.toxml(), 'use_sim_time': True}]
    )

    # 5. Gazebo에 로봇 소환 (Spawn)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            # '-world', 'drobot_world',  <-- 이 줄을 삭제하거나 주석 처리하세요!
            '-topic', 'robot_description',
            '-name', 'drobot',
            '-allow_renaming', 'true',
            '-z', '0.5' 
        ],
        output='screen'
    )

    # 6. Bridge 설정 (ROS 2 -> Gazebo 명령 전달)
    # 6. Bridge 설정 수정
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 1. 조종 명령: Gazebo가 실제로 듣고 있는 '/model/drobot/cmd_vel'로 직접 전송
            '/model/drobot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # 2. 오도메트리: 로봇의 위치 정보
            '/model/drobot/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # 3. 레이저 스캔: Gazebo 목록에 /scan으로 떠있었으므로 그대로 사용
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # 4. 시뮬레이션 시간
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        remappings=[
            # Gazebo의 복잡한 이름을 ROS 2에서는 깔끔하게 /cmd_vel, /odom으로 리매핑
            ('/model/drobot/cmd_vel', '/cmd_vel'),
            ('/model/drobot/odom', '/odom')
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        world_arg,
        gazebo,
        rsp,
        spawn
    ])