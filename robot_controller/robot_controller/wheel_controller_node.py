#!/usr/bin/env python3
"""
바퀴 제어 노드 (Wheel Controller Node)
- /cmd_vel 토픽을 받아서 바퀴 모터에 PWM 명령 전달
- Differential drive 방식
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32MultiArray


class WheelControllerNode(Node):
    def __init__(self):
        super().__init__('wheel_controller')

        self.enabled = True
        self.max_pwm = 255  # PWM 최대값

        # Parameters 선언
        self.declare_parameter('wheel_base', 0.3)  # 바퀴 간 거리 (m)
        self.declare_parameter('max_linear_vel', 1.0)  # 최대 선속도 (m/s)
        self.declare_parameter('max_angular_vel', 2.0)  # 최대 각속도 (rad/s)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.enable_sub = self.create_subscription(
            String, '/wheel/enable', self.enable_callback, 10)

        # Publishers
        self.motor_pub = self.create_publisher(Int32MultiArray, '/wheel/motor_pwm', 10)
        self.status_pub = self.create_publisher(String, '/wheel/status', 10)

        # Status timer (1Hz)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(f'Wheel Controller initialized (wheel_base: {self.wheel_base}m)')

    def enable_callback(self, msg):
        """바퀴 활성화/비활성화"""
        if msg.data.upper() == 'ENABLE':
            self.enabled = True
            self.get_logger().info('Wheels ENABLED')
        elif msg.data.upper() == 'DISABLE':
            self.enabled = False
            self.stop_wheels()
            self.get_logger().info('Wheels DISABLED')

    def cmd_vel_callback(self, msg):
        """속도 명령 처리 - Differential Drive 계산"""
        if not self.enabled:
            return

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # 속도 제한
        linear_x = max(-self.max_linear_vel, min(self.max_linear_vel, linear_x))
        angular_z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_z))

        # Differential drive 계산
        # left_vel = v - (ω * L / 2)
        # right_vel = v + (ω * L / 2)
        left_vel = linear_x - (angular_z * self.wheel_base / 2.0)
        right_vel = linear_x + (angular_z * self.wheel_base / 2.0)

        # PWM 값으로 변환 (-255 ~ 255)
        left_pwm = int((left_vel / self.max_linear_vel) * self.max_pwm)
        right_pwm = int((right_vel / self.max_linear_vel) * self.max_pwm)

        # 범위 제한
        left_pwm = max(-self.max_pwm, min(self.max_pwm, left_pwm))
        right_pwm = max(-self.max_pwm, min(self.max_pwm, right_pwm))

        # 모터 명령 발행
        motor_msg = Int32MultiArray()
        motor_msg.data = [left_pwm, right_pwm]
        self.motor_pub.publish(motor_msg)

        self.get_logger().debug(f'Motor PWM: L={left_pwm}, R={right_pwm}')

    def stop_wheels(self):
        """바퀴 정지"""
        motor_msg = Int32MultiArray()
        motor_msg.data = [0, 0]
        self.motor_pub.publish(motor_msg)
        self.get_logger().info('Wheels stopped')

    def publish_status(self):
        """상태 발행"""
        status_msg = String()
        status_msg.data = 'ENABLED' if self.enabled else 'DISABLED'
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_wheels()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
