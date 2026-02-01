"""
Collision detection and avoidance.

Real-time obstacle detection using LiDAR data and reactive avoidance.
"""
from typing import Tuple, Optional, Callable
import math
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.publisher import Publisher

from drobot_simulation.config import Config


class CollisionHandler:
    """Handles collision detection and avoidance maneuvers."""

    def __init__(
        self,
        cmd_pub: Publisher,
        logger: Optional[Callable] = None
    ):
        """
        Initialize collision handler.

        Args:
            cmd_pub: Publisher for velocity commands
            logger: Optional logger function (node.get_logger().warn, etc.)
        """
        self.cmd_pub = cmd_pub
        self.logger = logger or (lambda msg: None)
        self.last_scan: Optional[LaserScan] = None
        self.last_collision_time = 0.0

    def update_scan(self, scan: LaserScan) -> None:
        """
        Update stored LiDAR scan data.

        Args:
            scan: Latest LaserScan message
        """
        self.last_scan = scan

    def analyze_front_obstacles(self) -> Tuple[bool, float, float]:
        """
        Analyze obstacles in front of the robot.

        Returns:
            Tuple of (collision_imminent, min_distance, obstacle_direction)
        """
        if self.last_scan is None:
            return False, float('inf'), 0.0

        ranges = list(self.last_scan.ranges)
        num_ranges = len(ranges)
        angle_min = self.last_scan.angle_min
        angle_increment = self.last_scan.angle_increment

        # Calculate front cone indices (centered at 0 degrees)
        front_center = int((0 - angle_min) / angle_increment)
        half_cone = int(Config.COLLISION_CHECK_ANGLE / angle_increment)

        front_start = max(0, front_center - half_cone)
        front_end = min(num_ranges - 1, front_center + half_cone)

        # Extract front range data
        front_ranges = ranges[front_start:front_end + 1]

        # Filter valid readings
        valid_data = []
        for i, r in enumerate(front_ranges):
            if self.last_scan.range_min < r < self.last_scan.range_max:
                angle = angle_min + (front_start + i) * angle_increment
                valid_data.append((r, angle))

        if not valid_data:
            return False, float('inf'), 0.0

        # Find minimum distance and direction
        min_dist = float('inf')
        min_angle = 0.0
        for r, angle in valid_data:
            if r < min_dist:
                min_dist = r
                min_angle = angle

        collision_imminent = min_dist < Config.COLLISION_STOP_DIST
        return collision_imminent, min_dist, min_angle

    def get_side_clearance(self) -> Tuple[float, float]:
        """
        Get clearance on left and right sides.

        Returns:
            Tuple of (left_clearance, right_clearance) in meters
        """
        if self.last_scan is None:
            return 1.0, 1.0

        ranges = list(self.last_scan.ranges)
        num_ranges = len(ranges)
        sample_range = 20  # Number of samples to average

        # Left side (90 degrees)
        left_idx = num_ranges // 4
        left_data = ranges[max(0, left_idx - sample_range):left_idx + sample_range]
        left_valid = [r for r in left_data if 0.1 < r < 12.0]
        left_clearance = sum(left_valid) / len(left_valid) if left_valid else 0.0

        # Right side (-90 degrees)
        right_idx = 3 * num_ranges // 4
        right_data = ranges[max(0, right_idx - sample_range):min(num_ranges, right_idx + sample_range)]
        right_valid = [r for r in right_data if 0.1 < r < 12.0]
        right_clearance = sum(right_valid) / len(right_valid) if right_valid else 0.0

        return left_clearance, right_clearance

    def check_and_handle_collision(self, is_navigating: bool) -> bool:
        """
        Check for imminent collision and handle if needed.

        Args:
            is_navigating: Whether robot is currently navigating

        Returns:
            True if collision was handled (navigation should be cancelled)
        """
        if self.last_scan is None:
            return False

        # Cooldown check
        current_time = time.time()
        if current_time - self.last_collision_time < Config.COLLISION_COOLDOWN:
            return False

        # Only check during navigation
        if not is_navigating:
            return False

        collision_imminent, min_dist, obstacle_direction = self.analyze_front_obstacles()

        if collision_imminent:
            self.last_collision_time = current_time
            self._execute_avoidance(min_dist, obstacle_direction)
            return True

        return False

    def _execute_avoidance(self, min_dist: float, obstacle_direction: float) -> None:
        """
        Execute collision avoidance maneuver.

        Args:
            min_dist: Distance to nearest obstacle
            obstacle_direction: Direction to obstacle (radians)
        """
        self.logger(
            f'COLLISION WARNING! Distance: {min_dist:.2f}m, '
            f'Direction: {math.degrees(obstacle_direction):.0f}°'
        )

        twist = Twist()

        # 1. Emergency stop
        self.cmd_pub.publish(twist)
        time.sleep(0.2)

        # 2. Gentle backup
        twist.linear.x = Config.BACKUP_SPEED
        for _ in range(6):  # 0.6 seconds
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

        # 3. Stop and stabilize
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(0.3)

        # 4. Turn away from obstacle
        turn_direction = -1.0 if obstacle_direction > 0 else 1.0

        # Prefer side with more clearance
        left_clearance, right_clearance = self.get_side_clearance()
        if left_clearance > right_clearance + 0.3:
            turn_direction = 1.0
        elif right_clearance > left_clearance + 0.3:
            turn_direction = -1.0

        twist.angular.z = turn_direction * Config.ROTATION_SPEED
        for _ in range(8):  # 0.8 seconds
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

        # 5. Final stop
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(0.3)

    def is_in_cooldown(self) -> bool:
        """Check if collision handler is in cooldown period."""
        return (time.time() - self.last_collision_time) < Config.COLLISION_COOLDOWN
