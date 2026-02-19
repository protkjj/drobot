#!/usr/bin/env python3
"""Keyboard teleop node for Drobot."""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
Drobot Keyboard Teleop
----------------------
Move:
  w/s : forward/backward
  a/d : turn left/right

Other:
  x   : stop
  q   : quit
"""


class KeyboardTeleop(Node):
    """Read keyboard input and publish Twist to /cmd_vel."""

    def __init__(self) -> None:
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.9)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)

        self.get_logger().info(HELP)
        self.get_logger().info(
            f'Publishing /cmd_vel with linear={self.linear_speed:.2f}, '
            f'angular={self.angular_speed:.2f}'
        )

    def publish_cmd(self, linear_x: float, angular_z: float) -> None:
        """Publish one Twist command."""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.publisher.publish(cmd)


def get_key(input_stream, timeout: float = 0.1) -> str:
    """Read one key in raw terminal mode, non-blocking with timeout."""
    readable, _, _ = select.select([input_stream], [], [], timeout)
    if readable:
        return input_stream.read(1)
    return ''


def main(args=None) -> None:
    """Run keyboard teleop loop."""
    rclpy.init(args=args)
    node = KeyboardTeleop()

    tty_file = None
    try:
        tty_file = open('/dev/tty', 'r')
    except OSError:
        node.get_logger().error(
            'No controlling terminal available. '
            'Run teleop from an interactive shell.'
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    settings = termios.tcgetattr(tty_file)
    tty.setraw(tty_file.fileno())

    try:
        while rclpy.ok():
            key = get_key(tty_file)
            if not key:
                continue

            if key == 'w':
                node.publish_cmd(node.linear_speed, 0.0)
            elif key == 's':
                node.publish_cmd(-node.linear_speed, 0.0)
            elif key == 'a':
                node.publish_cmd(0.0, node.angular_speed)
            elif key == 'd':
                node.publish_cmd(0.0, -node.angular_speed)
            elif key == 'x':
                node.publish_cmd(0.0, 0.0)
            elif key == 'q':
                node.publish_cmd(0.0, 0.0)
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(tty_file, termios.TCSADRAIN, settings)
        tty_file.close()
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
