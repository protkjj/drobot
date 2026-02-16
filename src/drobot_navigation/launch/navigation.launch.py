import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue 
from ros_gz_bridge.actions import RosGzBridge
from launch.substitutions import Command

def launch_setup(context, *args, **kwargs):
    # 1. 경로 설정 (파이썬 문자열로 직접 처리해서 에러 방지)
    pkg_nav = get_package_share_directory('drobot_navigation')
    pkg_desc = get_package_share_directory('drobot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    

    world_name = LaunchConfiguration('world').perform(context)
    world_file = os.path.join(pkg_desc, 'worlds', f'{world_name}.sdf')
    
    # 2. Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 3. 로봇 소환
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'drobot', '-topic', 'robot_description', '-z', '0.1'],
    )

    # 4. 브릿지 (bridge_name 필수 추가, output 제거)
    bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=os.path.join(pkg_nav, 'config', 'ros_gz_bridge.yaml')
    )

    # 5. SLAM & EKF (기존 설정 유지)
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        parameters=[os.path.join(pkg_nav, 'config', 'slam_params.yaml'), {'use_sim_time': True}]
    )
    # URDF/Xacro 파일 경로 (육체 패키지에서 가져옴)
    urdf_file = os.path.join(pkg_desc, 'urdf', 'drobot.urdf.xacro')

    # 6. Robot State Publisher 노드 추가
    # Robot State Publisher 노드 수정 ✅
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str),
            'use_sim_time': True
        }]
    )

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 1. Nav2 파라미터 파일 설정
    nav2_params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    # 2. Nav2 Bringup 포함 (자율주행 핵심 엔진)
    # SLAM이 켜져 있으므로 slam:=True 옵션을 줍니다.
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_params_file,
            'autostart': 'True'
        }.items()
    )

    # 3. RViz2 (Nav2용 기본 설정파일 사용)
    rviz_config_file = os.path.join(pkg_nav, 'config', 'rviz_config.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 모든 노드 통합 반환
    return [
        gazebo,      # 세상 열기 (Sim)
        robot_state_publisher, # 로봇 설계도
        spawn_robot, # 로봇 소환 (Spawn)
        bridge,      # 통신 연결
        slam,        # 지도 그리기 (SLAM)
        nav2_bringup, # 목표 지점 이동 (Nav2) ✅ 추가됨
        rviz2        # 화면 확인
    ]

    

def generate_launch_description():
    pkg_desc = get_package_share_directory('drobot_description')
    pkg_share_path = os.path.dirname(pkg_desc)
    
    return LaunchDescription([
        # 환경 변수 설정
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                pkg_share_path,    # 1순위: 패키지들이 모여있는 share 폴더 (model:// 주소 해결용)
                ':',
                pkg_desc           # 2순위: 패키지 내부 폴더
            ]
        ),
        DeclareLaunchArgument('world', default_value='f1_circuit'),
        OpaqueFunction(function=launch_setup)
    ])