import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class FakeDiffOdom(Node):
    def __init__(self) -> None:
        super().__init__('fake_diff_odom')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('wheel_radius', 0.112)
        self.declare_parameter('wheel_separation', 0.35)
        self.declare_parameter('servo_step_rad', 0.05)
        self.declare_parameter('servo_min_rad', -3.14)
        self.declare_parameter('servo_max_rad', 3.14)
        self.declare_parameter('arm_step_rad', 0.05)
        self.declare_parameter('arm_min_rad', -3.14)
        self.declare_parameter('arm_max_rad', 3.14)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.servo_step = float(self.get_parameter('servo_step_rad').value)
        self.servo_min = float(self.get_parameter('servo_min_rad').value)
        self.servo_max = float(self.get_parameter('servo_max_rad').value)
        self.arm_step = float(self.get_parameter('arm_step_rad').value)
        self.arm_min = float(self.get_parameter('arm_min_rad').value)
        self.arm_max = float(self.get_parameter('arm_max_rad').value)

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.joint_cmd_sub = self.create_subscription(String, '/joint_key_cmd', self.joint_cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        self.vx = 0.0
        self.vth = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.servo_lf = 0.0
        self.arm_left = 0.0
        self.wheel_lf = 0.0
        self.wheel_lr = 0.0
        self.wheel_rf = 0.0
        self.wheel_rr = 0.0

        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self.update)

    def cmd_cb(self, msg: Twist) -> None:
        self.vx = float(msg.linear.x)
        self.vth = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)

    def joint_cmd_cb(self, msg: String) -> None:
        cmd = msg.data.strip().lower()

        if cmd == 'servo_up':
            self.servo_lf = self._clamp(self.servo_lf + self.servo_step, self.servo_min, self.servo_max)
        elif cmd == 'servo_down':
            self.servo_lf = self._clamp(self.servo_lf - self.servo_step, self.servo_min, self.servo_max)
        elif cmd == 'arm_up':
            self.arm_left = self._clamp(self.arm_left + self.arm_step, self.arm_min, self.arm_max)
        elif cmd == 'arm_down':
            self.arm_left = self._clamp(self.arm_left - self.arm_step, self.arm_min, self.arm_max)
        elif cmd == 'reset_joints':
            self.servo_lf = 0.0
            self.arm_left = 0.0

    def update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now
        if dt <= 0.0:
            return

        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout_sec:
            vx = 0.0
            vth = 0.0
        else:
            vx = self.vx
            vth = self.vth

        self.x += vx * math.cos(self.yaw) * dt
        self.y += vx * math.sin(self.yaw) * dt
        self.yaw += vth * dt

        left_v = vx - 0.5 * self.wheel_separation * vth
        right_v = vx + 0.5 * self.wheel_separation * vth
        left_w = left_v / self.wheel_radius
        right_w = right_v / self.wheel_radius

        self.wheel_lf += left_w * dt
        self.wheel_lr += left_w * dt
        self.wheel_rf += right_w * dt
        self.wheel_rr += right_w * dt

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)

        joint = JointState()
        joint.header.stamp = now.to_msg()
        joint.name = [
            'servo_LF',
            'armjoint_left',
            'wheejoint_LF',
            'wheeljoint_LR',
            'wheeljoint_RF',
            'wheeljoint_RR',
        ]
        joint.position = [
            self.servo_lf,
            self.arm_left,
            self.wheel_lf,
            self.wheel_lr,
            self.wheel_rf,
            self.wheel_rr,
        ]
        joint.velocity = [
            0.0,
            0.0,
            left_w,
            left_w,
            right_w,
            right_w,
        ]
        self.joint_pub.publish(joint)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeDiffOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
