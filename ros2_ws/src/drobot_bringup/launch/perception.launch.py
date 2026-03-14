import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('perception')

    # Ensure this path matches where you put your .pt file in the package
    model_path = os.path.join(pkg_share, 'models', 'cone.pt')

    return LaunchDescription([
        Node(
            package='perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[
                {'model_path': model_path},
                {'debug_mode': True}
            ]
        ),
        Node(
            package='drobot_tracker',
            executable='tracker_node',
            name='object_tracker_node',
            output='screen',
            parameters=[
                {'gate_threshold': 2.0},
                {'max_misses': 5},
                {'min_hits': 3},
                {'process_noise_std': 0.5},
                {'measurement_noise_std': 0.3},
            ]
        ),
    ])
