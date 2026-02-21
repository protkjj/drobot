import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


HELP = """
Joint key teleop
  r: servo up
  f: servo down
  t: arm up
  g: arm down
  space: reset servo/arm to zero
  q: quit
"""


class JointKeyTeleop(Node):
    def __init__(self) -> None:
        super().__init__('joint_key_teleop')
        self.pub = self.create_publisher(String, '/joint_key_cmd', 10)
        self.bindings = {
            'r': 'servo_up',
            'f': 'servo_down',
            't': 'arm_up',
            'g': 'arm_down',
            ' ': 'reset_joints',
        }

    def publish_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)


def _get_key(timeout_sec: float = 0.1) -> str:
    dr, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if dr:
        return sys.stdin.read(1)
    return ''


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointKeyTeleop()

    print(HELP)
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    try:
        while rclpy.ok():
            key = _get_key()
            if not key:
                continue
            if key == 'q':
                break
            cmd = node.bindings.get(key)
            if cmd:
                node.publish_cmd(cmd)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

