#!/usr/bin/env python3
"""
고수준 명령 노드 (Commander Node)
- 사용자 입력 또는 자율 미션에 따라 로봇 모드 및 동작 제어
- 키보드 입력으로 테스트 가능
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty


class CommanderNode(Node):
    def __init__(self):
        super().__init__('commander')

        # Publishers
        self.mode_cmd_pub = self.create_publisher(String, '/mode_command', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.flight_cmd_pub = self.create_publisher(Twist, '/flight/cmd_vel', 10)

        # Subscribers
        self.mode_sub = self.create_subscription(
            String, '/robot_mode', self.mode_callback, 10)

        self.current_mode = 'GROUND'

        # 속도 설정
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        self.vertical_speed = 0.5  # m/s (비행 시)

        self.get_logger().info('Commander Node initialized')
        self.print_help()

    def mode_callback(self, msg):
        """현재 모드 업데이트"""
        self.current_mode = msg.data

    def print_help(self):
        """도움말 출력"""
        help_text = """
========================================
       Wheeled Drone Commander
========================================
Mode Commands:
  t : Takeoff (GROUND -> FLIGHT)
  l : Land (FLIGHT -> GROUND)

Ground Mode (WASD):
  w/s : Forward / Backward
  a/d : Turn Left / Turn Right
  x   : Stop

Flight Mode (WASD + QE):
  w/s : Forward / Backward
  a/d : Strafe Left / Right
  q/e : Turn Left / Right
  r/f : Up / Down
  x   : Hover (stop)

Other:
  h : Show this help
  Ctrl+C : Exit
========================================
"""
        print(help_text)

    def send_mode_command(self, command):
        """모드 전환 명령 전송"""
        msg = String()
        msg.data = command
        self.mode_cmd_pub.publish(msg)
        self.get_logger().info(f'Sent mode command: {command}')

    def send_ground_velocity(self, linear_x, angular_z):
        """지상 속도 명령 전송"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def send_flight_velocity(self, vx, vy, vz, yaw_rate):
        """비행 속도 명령 전송"""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = yaw_rate
        self.flight_cmd_pub.publish(msg)

    def process_key(self, key):
        """키 입력 처리"""
        if key == 't':
            self.send_mode_command('TAKEOFF')
        elif key == 'l':
            self.send_mode_command('LAND')
        elif key == 'h':
            self.print_help()
        elif self.current_mode == 'GROUND':
            self.process_ground_key(key)
        elif self.current_mode == 'FLIGHT':
            self.process_flight_key(key)

    def process_ground_key(self, key):
        """지상 모드 키 처리"""
        if key == 'w':
            self.send_ground_velocity(self.linear_speed, 0.0)
        elif key == 's':
            self.send_ground_velocity(-self.linear_speed, 0.0)
        elif key == 'a':
            self.send_ground_velocity(0.0, self.angular_speed)
        elif key == 'd':
            self.send_ground_velocity(0.0, -self.angular_speed)
        elif key == 'x':
            self.send_ground_velocity(0.0, 0.0)

    def process_flight_key(self, key):
        """비행 모드 키 처리"""
        if key == 'w':
            self.send_flight_velocity(self.linear_speed, 0.0, 0.0, 0.0)
        elif key == 's':
            self.send_flight_velocity(-self.linear_speed, 0.0, 0.0, 0.0)
        elif key == 'a':
            self.send_flight_velocity(0.0, self.linear_speed, 0.0, 0.0)
        elif key == 'd':
            self.send_flight_velocity(0.0, -self.linear_speed, 0.0, 0.0)
        elif key == 'q':
            self.send_flight_velocity(0.0, 0.0, 0.0, self.angular_speed)
        elif key == 'e':
            self.send_flight_velocity(0.0, 0.0, 0.0, -self.angular_speed)
        elif key == 'r':
            self.send_flight_velocity(0.0, 0.0, self.vertical_speed, 0.0)
        elif key == 'f':
            self.send_flight_velocity(0.0, 0.0, -self.vertical_speed, 0.0)
        elif key == 'x':
            self.send_flight_velocity(0.0, 0.0, 0.0, 0.0)


def get_key(settings):
    """터미널에서 키 입력 받기"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = CommanderNode()

    # 터미널 설정 저장
    settings = termios.tcgetattr(sys.stdin)

    try:
        while rclpy.ok():
            key = get_key(settings)
            if key:
                node.process_key(key)
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        # 터미널 설정 복원
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
