from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # model_path: 모델 파일명 (cone.pt, best.pt 등)
    # conf_threshold: YOLO 추론 임계값
    # post_filter_conf: 후처리 confidence 임계값
    # max_detect_distance: 최대 감지 거리 (m)

    return LaunchDescription([
        Node(
            package='perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[
                {'model_path': 'cone.pt'},
                {'conf_threshold': 0.9},
                {'post_filter_conf': 0.7},
                {'max_detect_distance': 5.0},
                {'debug_mode': True},
            ]
        ),
        # Node(
        #     package='perception',
        #     executable='perception_node',
        #     name='perception_node_best',
        #     output='screen',
        #     parameters=[
        #         {'model_path': 'best.pt'},
        #         {'conf_threshold': 0.6},
        #         {'post_filter_conf': 0.5},
        #         {'max_detect_distance': 5.0},
        #         {'debug_mode': True},
        #     ]
        # ),
    ])