#!/usr/bin/env python3
"""
Manual controller node.

Consumes teleop Twist commands and exposes lightweight controller status.
"""
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class ManualController(Node):
    """Tracks incoming /cmd_vel messages from teleop."""

    def __init__(self):
        super().__init__('manual_controller')

        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        self.status_pub = self.create_publisher(Bool, '/drobot_controller/manual_active', 10)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info('manual_controller ready: listening on /cmd_vel')

    def _cmd_vel_callback(self, msg: Twist):
        self.last_linear_x = msg.linear.x
        self.last_angular_z = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def _publish_status(self):
        now = self.get_clock().now()
        delta = (now - self.last_cmd_time).nanoseconds / 1e9
        is_active = delta < 1.0
        self.status_pub.publish(Bool(data=is_active))

        self.get_logger().debug(
            f'active={is_active} linear_x={self.last_linear_x:.2f} angular_z={self.last_angular_z:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ManualController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
