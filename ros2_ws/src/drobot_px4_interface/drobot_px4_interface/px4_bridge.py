#!/usr/bin/env python3
"""
PX4 uORB ↔ ROS 2 토픽 번역 노드

/drobot/mode_cmd → VehicleCommand (CUSTOM_0)                  모드 전환
/cmd_vel         → OffboardControlMode + TrajectorySetpoint    드론 속도 제어
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
)

_NAN = float("nan")

# Micro XRCE-DDS 기본 QoS
_PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class PX4Bridge(Node):

    MODE_ROVER = 0
    MODE_DRONE = 1

    def __init__(self):
        super().__init__("px4_bridge")

        # ── PX4 Publishers ────────────────────────────────
        self._pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", _PX4_QOS
        )
        self._pub_offboard = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", _PX4_QOS
        )
        self._pub_setpoint = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", _PX4_QOS
        )

        # ── ROS 2 Subscribers ─────────────────────────────
        self.create_subscription(Int32, "/drobot/mode_cmd", self._on_mode_cmd, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        # ── State ─────────────────────────────────────────
        self._mode = self.MODE_ROVER
        self._offboard_counter = 0

        # Offboard heartbeat — 10 Hz
        self.create_timer(0.1, self._tick)

        self.get_logger().info("PX4 Bridge started")

    # ─── timestamp (microseconds) ─────────────────────────
    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    # ─── 모드 전환 ────────────────────────────────────────
    def _on_mode_cmd(self, msg: Int32):
        mode = msg.data
        self._publish_command(
            VehicleCommand.VEHICLE_CMD_CUSTOM_0, param1=float(mode)
        )

        if mode == self.MODE_DRONE:
            self._mode = self.MODE_DRONE
            self._offboard_counter = 0
            self.get_logger().info("Mode → DRONE")
        else:
            self._mode = self.MODE_ROVER
            self._disarm()
            self.get_logger().info("Mode → ROVER")

    # ─── cmd_vel → TrajectorySetpoint (NED 속도) ──────────
    def _on_cmd_vel(self, msg: Twist):
        if self._mode != self.MODE_DRONE:
            return

        sp = TrajectorySetpoint()
        sp.timestamp = self._now_us()
        # ROS FLU → PX4 NED
        sp.velocity[0] = msg.linear.x
        sp.velocity[1] = -msg.linear.y
        sp.velocity[2] = -msg.linear.z
        sp.yawspeed = -msg.angular.z
        sp.position = [_NAN, _NAN, _NAN]
        sp.yaw = _NAN
        self._pub_setpoint.publish(sp)

    # ─── Offboard heartbeat (10 Hz) ──────────────────────
    def _tick(self):
        if self._mode != self.MODE_DRONE:
            return

        ocm = OffboardControlMode()
        ocm.timestamp = self._now_us()
        ocm.position = False
        ocm.velocity = True
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self._pub_offboard.publish(ocm)

        self._offboard_counter += 1
        if self._offboard_counter == 10:
            self._set_offboard_mode()
            self._arm()

    # ─── VehicleCommand helpers ───────────────────────────
    def _publish_command(self, command: int, **params):
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        for key, val in params.items():
            setattr(msg, key, val)
        self._pub_cmd.publish(msg)

    def _arm(self):
        self._publish_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        self.get_logger().info("ARM")

    def _disarm(self):
        self._publish_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )
        self.get_logger().info("DISARM")

    def _set_offboard_mode(self):
        self._publish_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )
        self.get_logger().info("OFFBOARD mode set")


def main(args=None):
    rclpy.init(args=args)
    node = PX4Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
