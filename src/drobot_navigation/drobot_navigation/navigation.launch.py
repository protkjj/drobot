import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 경로 설정
    nav_pkg = get_package_share_directory('drobot_navigation')
    sim_pkg = get_package_share_directory('drobot_description') # 로봇 모델 패키지
    
    # 설정 파일 경로
    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    
    # 2. Gazebo 시뮬레이션 실행 (drobot_description에 있는 시뮬레이션 런치 호출)
    # ※ 주의: drobot_description에 simulation.launch.py가 있어야 합니다.
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'simulation.launch.py')
        )
    )

    # 3. SLAM Toolbox (지도 작성)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4. Nav2 Stack 실행 (Controller, Planner 등)
    nav2_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'True'
        }.items()
    )

    # 5. 내가 만든 두뇌 (Goal Navigator)
    goal_navigator = Node(
        package='drobot_navigation',
        executable='goal_navigator', # setup.py에 등록한 이름
        name='goal_navigator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        simulation,
        slam_node,
        nav2_nodes,
        goal_navigator
    ])
