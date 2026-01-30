#!/usr/bin/env python3
"""
모드 관리 노드 (Mode Manager Node)
- GROUND: 지상 주행 모드 (바퀴 사용)
- FLIGHT: 비행 모드 (프로펠러 사용)
- TRANSITION: 전환 중
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from enum import Enum


class RobotMode(Enum):
    GROUND = 0
    TRANSITION_TO_FLIGHT = 1
    FLIGHT = 2
    TRANSITION_TO_GROUND = 3


class ModeManagerNode(Node):
    def __init__(self):
        super().__init__('mode_manager')

        self.current_mode = RobotMode.GROUND

        # Publishers
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        self.wheel_enable_pub = self.create_publisher(String, '/wheel/enable', 10)
        self.flight_enable_pub = self.create_publisher(String, '/flight/enable', 10)

        # Subscribers
        self.mode_cmd_sub = self.create_subscription(
            String, '/mode_command', self.mode_command_callback, 10)

        # Timer for publishing current mode (10Hz)
        self.timer = self.create_timer(0.1, self.publish_mode)

        self.get_logger().info('Mode Manager initialized - Starting in GROUND mode')

    def mode_command_callback(self, msg):
        """모드 전환 명령 처리"""
        command = msg.data.upper()

        if command == 'TAKEOFF' and self.current_mode == RobotMode.GROUND:
            self.transition_to_flight()
        elif command == 'LAND' and self.current_mode == RobotMode.FLIGHT:
            self.transition_to_ground()
        elif command == 'GROUND' and self.current_mode != RobotMode.GROUND:
            self.transition_to_ground()
        elif command == 'FLIGHT' and self.current_mode != RobotMode.FLIGHT:
            self.transition_to_flight()
        else:
            self.get_logger().warn(f'Invalid command "{command}" for current mode {self.current_mode.name}')

    def transition_to_flight(self):
        """지상 -> 비행 모드 전환"""
        self.get_logger().info('Transitioning to FLIGHT mode...')
        self.current_mode = RobotMode.TRANSITION_TO_FLIGHT

        # 1. 바퀴 정지
        wheel_msg = String()
        wheel_msg.data = 'DISABLE'
        self.wheel_enable_pub.publish(wheel_msg)

        # 2. 비행 모드 활성화
        flight_msg = String()
        flight_msg.data = 'ENABLE'
        self.flight_enable_pub.publish(flight_msg)

        self.current_mode = RobotMode.FLIGHT
        self.get_logger().info('Now in FLIGHT mode')

    def transition_to_ground(self):
        """비행 -> 지상 모드 전환"""
        self.get_logger().info('Transitioning to GROUND mode...')
        self.current_mode = RobotMode.TRANSITION_TO_GROUND

        # 1. 비행 모드 비활성화 (착륙)
        flight_msg = String()
        flight_msg.data = 'DISABLE'
        self.flight_enable_pub.publish(flight_msg)

        # 2. 바퀴 활성화
        wheel_msg = String()
        wheel_msg.data = 'ENABLE'
        self.wheel_enable_pub.publish(wheel_msg)

        self.current_mode = RobotMode.GROUND
        self.get_logger().info('Now in GROUND mode')

    def publish_mode(self):
        """현재 모드 발행"""
        msg = String()
        msg.data = self.current_mode.name
        self.mode_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
