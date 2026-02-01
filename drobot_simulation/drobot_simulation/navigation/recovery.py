"""
Recovery behaviors for stuck and oscillation situations.

Provides recovery maneuvers when exploration gets stuck.
"""
from typing import Optional, Callable
import random
import time
from collections import deque

from geometry_msgs.msg import Twist
from rclpy.publisher import Publisher
from rclpy.node import Node
from nav2_msgs.srv import ClearEntireCostmap

from drobot_simulation.config import Config


class RecoveryManager:
    """Manages recovery behaviors for stuck situations."""

    def __init__(
        self,
        cmd_pub: Publisher,
        node: Node,
        logger: Optional[Callable] = None
    ):
        """
        Initialize recovery manager.

        Args:
            cmd_pub: Publisher for velocity commands
            node: ROS node for service clients
            logger: Optional logger function
        """
        self.cmd_pub = cmd_pub
        self.node = node
        self.logger = logger or (lambda msg: None)

        # Costmap clear services
        self.clear_global_costmap = node.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )
        self.clear_local_costmap = node.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )

        # Position history for oscillation detection
        self.position_history: deque = deque(maxlen=Config.POSITION_HISTORY_SIZE)

    def update_position(self, x: float, y: float) -> None:
        """
        Update position history for oscillation detection.

        Args:
            x, y: Current robot position
        """
        # Discretize to 0.5m grid
        current_pos = (round(x * 2) / 2, round(y * 2) / 2)
        self.position_history.append(current_pos)

    def detect_oscillation(self, min_goals: int = 1) -> bool:
        """
        Detect if robot is oscillating between positions.

        Args:
            min_goals: Minimum goals attempted before detection

        Returns:
            True if oscillation detected
        """
        # Don't detect oscillation before attempting goals
        if min_goals < 2:
            return False

        if len(self.position_history) < 15:
            return False

        recent_positions = list(self.position_history)[-15:]

        # Need at least 3 unique positions to check for A-B pattern
        unique_positions = set(recent_positions)
        if len(unique_positions) == 1:
            # Stationary - not oscillation, just stuck
            return False

        # Check A-B-A-B pattern (true oscillation between 2 positions)
        if len(unique_positions) == 2:
            # Count transitions between positions
            transitions = 0
            for i in range(1, len(recent_positions)):
                if recent_positions[i] != recent_positions[i-1]:
                    transitions += 1
            # True oscillation has many transitions (A->B->A->B...)
            if transitions >= 6:
                self.logger(f'A-B oscillation pattern detected! {transitions} transitions')
                return True

        return False

    def backup(
        self,
        speed: float = Config.RECOVERY_BACKUP_SPEED,
        duration: float = 1.0
    ) -> None:
        """
        Execute backup maneuver.

        Args:
            speed: Backup speed (negative for reverse)
            duration: Duration in seconds
        """
        self.logger('Recovery: Backing up...')
        twist = Twist()
        twist.linear.x = speed

        iterations = int(duration / 0.1)
        for _ in range(iterations):
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)

    def rotate(
        self,
        speed: float = Config.RECOVERY_ROTATION_SPEED,
        duration: float = 1.5
    ) -> None:
        """
        Execute rotation maneuver.

        Args:
            speed: Rotation speed (positive = counter-clockwise)
            duration: Duration in seconds
        """
        twist = Twist()
        twist.angular.z = speed

        iterations = int(duration / 0.1)
        for _ in range(iterations):
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def clear_costmaps(self) -> None:
        """Clear both local and global costmaps."""
        self.logger('Clearing costmaps...')

        if self.clear_global_costmap.wait_for_service(timeout_sec=1.0):
            req = ClearEntireCostmap.Request()
            self.clear_global_costmap.call_async(req)

        if self.clear_local_costmap.wait_for_service(timeout_sec=1.0):
            req = ClearEntireCostmap.Request()
            self.clear_local_costmap.call_async(req)

        time.sleep(0.5)

    def handle_oscillation(self) -> None:
        """Execute aggressive recovery for oscillation."""
        self.logger('Handling oscillation - aggressive recovery')

        # Stronger backup
        self.backup(speed=Config.AGGRESSIVE_BACKUP_SPEED, duration=1.5)

        # Random rotation direction and duration
        rotation_dir = random.choice([-1, 1])
        rotation_time = random.uniform(1.0, 2.5)

        twist = Twist()
        twist.angular.z = rotation_dir * Config.AGGRESSIVE_ROTATION_SPEED
        start = time.time()
        while time.time() - start < rotation_time:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        # Clear costmaps
        self.clear_costmaps()

        # Clear position history
        self.position_history.clear()

        time.sleep(0.5)

    def full_recovery(self) -> None:
        """Execute full recovery sequence."""
        self.logger('Recovery: Rotating and clearing...')

        self.clear_costmaps()

        twist = Twist()
        twist.angular.z = Config.RECOVERY_ROTATION_SPEED
        for _ in range(15):  # 1.5 seconds
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        time.sleep(0.3)
        self.clear_costmaps()

    def reset_history(self) -> None:
        """Clear position history."""
        self.position_history.clear()
