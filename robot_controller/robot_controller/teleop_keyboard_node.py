#!/usr/bin/env python3
"""
RC카 키보드 조종 노드 (Teleop Keyboard Node)
- 키보드 입력을 받아 /cmd_vel 토픽으로 Twist 메시지 발행
"""
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# 키 바인딩 설정
MOVE_BINDINGS = {
    'w': (1.0, 0.0),    # 전진
    's': (-1.0, 0.0),   # 후진
    'a': (0.0, 1.0),    # 좌회전
    'd': (0.0, -1.0),   # 우회전
    'q': (1.0, 1.0),    # 전진 + 좌회전
    'e': (1.0, -1.0),   # 전진 + 우회전
    'z': (-1.0, 1.0),   # 후진 + 좌회전
    'c': (-1.0, -1.0),  # 후진 + 우회전
    ' ': (0.0, 0.0),    # 정지 (스페이스바)
    'x': (0.0, 0.0),    # 정지
}

SPEED_BINDINGS = {
    'r': (1.1, 1.0),    # 선속도 증가
    'f': (0.9, 1.0),    # 선속도 감소
    't': (1.0, 1.1),    # 각속도 증가
    'g': (1.0, 0.9),    # 각속도 감소
}


def get_key(settings):
    """키보드 입력 받기"""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_help():
    """도움말 출력"""
    help_msg = """
╔═══════════════════════════════════════════════════╗
║         RC카 키보드 컨트롤러                      ║
╠═══════════════════════════════════════════════════╣
║  이동 조작:                                       ║
║     q   w   e        (전진+좌회전, 전진, 전진+우회전)║
║     a   x   d        (좌회전, 정지, 우회전)        ║
║     z   s   c        (후진+좌회전, 후진, 후진+우회전)║
║                                                   ║
║  속도 조절:                                       ║
║     r/f : 선속도 증가/감소                        ║
║     t/g : 각속도 증가/감소                        ║
║                                                   ║
║  기타:                                            ║
║     스페이스바 : 긴급 정지                        ║
║     Ctrl+C : 종료                                 ║
╚═══════════════════════════════════════════════════╝
"""
    print(help_msg)


class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        # 파라미터 선언
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Teleop Keyboard Node started')
        self.get_logger().info(f'Linear speed: {self.linear_speed}, Angular speed: {self.angular_speed}')

    def publish_twist(self, linear_x: float, angular_z: float):
        """Twist 메시지 발행"""
        twist = Twist()
        twist.linear.x = linear_x * self.linear_speed
        twist.angular.z = angular_z * self.angular_speed
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        """정지 명령 발행"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    # 터미널 설정 저장
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = TeleopKeyboardNode()

    print_help()
    print(f'\n현재 속도 - 선속도: {node.linear_speed:.2f} m/s, 각속도: {node.angular_speed:.2f} rad/s')

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key in MOVE_BINDINGS:
                linear_x, angular_z = MOVE_BINDINGS[key]
                node.publish_twist(linear_x, angular_z)

                if linear_x == 0.0 and angular_z == 0.0:
                    print('\r정지                              ', end='')
                else:
                    print(f'\r이동: linear={linear_x * node.linear_speed:.2f}, angular={angular_z * node.angular_speed:.2f}    ', end='')

            elif key in SPEED_BINDINGS:
                linear_mult, angular_mult = SPEED_BINDINGS[key]
                node.linear_speed *= linear_mult
                node.angular_speed *= angular_mult

                # 속도 범위 제한
                node.linear_speed = max(0.1, min(2.0, node.linear_speed))
                node.angular_speed = max(0.1, min(4.0, node.angular_speed))

                print(f'\r속도 변경 - 선속도: {node.linear_speed:.2f} m/s, 각속도: {node.angular_speed:.2f} rad/s    ', end='')

            elif key == '\x03':  # Ctrl+C
                break

    except Exception as e:
        print(f'\n오류 발생: {e}')

    finally:
        # 정지 명령 전송
        node.stop()
        print('\n\n종료합니다...')

        # 터미널 설정 복원
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
