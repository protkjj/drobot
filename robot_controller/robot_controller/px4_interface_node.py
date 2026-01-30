#!/usr/bin/env python3
"""
PX4 인터페이스 노드 (PX4 Interface Node)
- Offboard 모드로 드론 제어
- MicroXRCE-DDS를 통해 PX4와 통신

주의: px4_msgs 패키지가 필요합니다.
설치: https://github.com/PX4/px4_msgs
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# PX4 메시지 임포트 (px4_msgs 패키지 필요)
try:
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleStatus,
        VehicleOdometry,
        VehicleLocalPosition
    )
    PX4_MSGS_AVAILABLE = True
except ImportError:
    PX4_MSGS_AVAILABLE = False


class PX4InterfaceNode(Node):
    def __init__(self):
        super().__init__('px4_interface')

        if not PX4_MSGS_AVAILABLE:
            self.get_logger().error('px4_msgs package not found! Install it first.')
            self.get_logger().error('git clone https://github.com/PX4/px4_msgs.git')
            return

        # QoS 설정 (PX4 호환)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.enabled = False
        self.armed = False
        self.offboard_mode = False
        self.offboard_setpoint_counter = 0

        # 현재 위치
        self.current_position = [0.0, 0.0, 0.0]

        # 목표 위치/속도 (NED 좌표계: z가 음수 = 위)
        self.target_position = [0.0, 0.0, -2.0]  # 기본 호버링 높이 2m
        self.target_velocity = [0.0, 0.0, 0.0]
        self.target_yaw = 0.0

        # ---------- Publishers to PX4 ----------
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # ---------- Subscribers from PX4 ----------
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_position_callback, qos_profile)

        # ---------- ROS 2 Interface ----------
        self.enable_sub = self.create_subscription(
            String, '/flight/enable', self.enable_callback, 10)
        self.flight_cmd_sub = self.create_subscription(
            Twist, '/flight/cmd_vel', self.flight_cmd_callback, 10)
        self.flight_position_sub = self.create_subscription(
            Twist, '/flight/target_position', self.position_cmd_callback, 10)

        # Status publisher
        self.status_pub = self.create_publisher(String, '/flight/status', 10)

        # Offboard 제어 타이머 (10Hz - PX4 요구사항)
        self.offboard_timer = self.create_timer(0.1, self.offboard_timer_callback)

        # Status 타이머 (1Hz)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('PX4 Interface initialized')

    def enable_callback(self, msg):
        """비행 모드 활성화/비활성화"""
        if msg.data.upper() == 'ENABLE':
            self.enabled = True
            self.offboard_setpoint_counter = 0
            self.get_logger().info('Flight mode ENABLED - Starting offboard sequence...')
        elif msg.data.upper() == 'DISABLE':
            self.enabled = False
            self.land()
            self.get_logger().info('Flight mode DISABLED - Landing...')

    def vehicle_status_callback(self, msg):
        """차량 상태 업데이트"""
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard_mode = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def local_position_callback(self, msg):
        """현재 위치 업데이트"""
        self.current_position = [msg.x, msg.y, msg.z]

    def flight_cmd_callback(self, msg):
        """속도 명령 처리"""
        if not self.enabled:
            return
        # Twist -> NED 속도 (x: 전진, y: 좌측, z: 상승)
        # NED에서 z는 아래가 양수이므로 부호 반전
        self.target_velocity = [msg.linear.x, -msg.linear.y, -msg.linear.z]

    def position_cmd_callback(self, msg):
        """위치 명령 처리"""
        if not self.enabled:
            return
        # Twist.linear을 위치로 사용 (x, y, z)
        # NED에서 z는 아래가 양수이므로 부호 반전
        self.target_position = [msg.linear.x, msg.linear.y, -msg.linear.z]
        self.target_yaw = msg.angular.z

    def offboard_timer_callback(self):
        """Offboard 제어 루프 (10Hz)"""
        if not self.enabled:
            return

        if not PX4_MSGS_AVAILABLE:
            return

        # Offboard 모드를 위해 setpoint를 계속 발행해야 함
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # 처음 10번 setpoint 발행 후 Offboard 모드 전환 및 Arm
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def publish_offboard_control_mode(self):
        """Offboard 제어 모드 발행"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        """궤적 setpoint 발행"""
        msg = TrajectorySetpoint()
        msg.position = self.target_position
        msg.velocity = [float('nan'), float('nan'), float('nan')]  # 위치 제어 시 NaN
        msg.yaw = self.target_yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """차량 명령 발행"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        """드론 Arm"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """드론 Disarm"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Offboard 모드 전환"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # base mode
            param2=6.0   # offboard mode
        )
        self.get_logger().info('Offboard mode command sent')

    def land(self):
        """착륙 명령"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Land command sent')

    def takeoff(self, altitude=2.0):
        """이륙 명령"""
        self.target_position = [
            self.current_position[0],
            self.current_position[1],
            -altitude  # NED 좌표계
        ]
        self.get_logger().info(f'Takeoff to {altitude}m')

    def publish_status(self):
        """상태 발행"""
        status = f'enabled={self.enabled}, armed={self.armed}, offboard={self.offboard_mode}'
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PX4InterfaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
