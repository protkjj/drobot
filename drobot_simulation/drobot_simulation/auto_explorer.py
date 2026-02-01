#!/usr/bin/env python3
"""
Auto Explorer Node v7 - Refactored

Modular frontier-based exploration with:
- Separated concerns (detection, scoring, collision, recovery)
- Configurable constants
- Type-safe data structures
- Memory management for long-running exploration
"""
import math
import time
from collections import deque
from typing import Optional, Tuple, List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses, ComputePathToPose
from action_msgs.msg import GoalStatus

from drobot_simulation.config import Config
from drobot_simulation.types import RobotState, MapInfo, ExplorationStats, NavigationState
from drobot_simulation.frontier import FrontierDetector, FrontierScorer
from drobot_simulation.navigation import CollisionHandler, RecoveryManager
from drobot_simulation import utils


class AutoExplorer(Node):
    """Autonomous frontier-based exploration node."""

    def __init__(self):
        super().__init__('auto_explorer')

        # Initialize components
        self._init_publishers_and_subscribers()
        self._init_action_clients()
        self._init_components()
        self._init_state()
        self._init_timers()

        self.get_logger().info('Auto Explorer v7 started! (Refactored)')

    def _init_publishers_and_subscribers(self):
        """Set up ROS publishers and subscribers."""
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)

    def _init_action_clients(self):
        """Set up Nav2 action clients."""
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_poses_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses'
        )
        self.compute_path_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose'
        )

    def _init_components(self):
        """Initialize modular components."""
        self.frontier_detector = FrontierDetector()
        self.frontier_scorer = FrontierScorer()
        self.collision_handler = CollisionHandler(
            self.cmd_pub,
            logger=self.get_logger().warn
        )
        self.recovery_manager = RecoveryManager(
            self.cmd_pub,
            self,
            logger=self.get_logger().info
        )

    def _init_state(self):
        """Initialize state tracking."""
        self.robot = RobotState()
        self.map_info = MapInfo()
        self.stats = ExplorationStats()
        self.nav_state = NavigationState()

        # Goal tracking
        self.failed_goals: set = set()
        self.unreachable_frontiers: set = set()
        self.current_distance_stage = 0

        # Repetition prevention
        self.last_target_key: Optional[Tuple[float, float]] = None
        self.same_target_count = 0
        self.last_goal_direction: Optional[float] = None

        # Progress tracking
        self.exploration_start_time: Optional[float] = None
        self.last_robot_pos: Optional[Tuple[float, float]] = None

    def _init_timers(self):
        """Set up periodic timers."""
        self.explore_timer = self.create_timer(
            Config.EXPLORE_INTERVAL, self._explore_callback
        )
        self.timeout_timer = self.create_timer(
            Config.TIMEOUT_CHECK_INTERVAL, self._check_timeout
        )
        self.collision_timer = self.create_timer(
            Config.COLLISION_CHECK_INTERVAL, self._check_collision
        )

    # ==================== Callbacks ====================

    def _odom_callback(self, msg):
        """Process odometry data."""
        self.robot.x = msg.pose.pose.position.x
        self.robot.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot.yaw = utils.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        # Update recovery manager position history
        self.recovery_manager.update_position(self.robot.x, self.robot.y)

    def _scan_callback(self, msg):
        """Process LiDAR data."""
        self.collision_handler.update_scan(msg)

    def _map_callback(self, msg):
        """Process map updates."""
        self.map_info.data = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width)
        )
        self.map_info.resolution = msg.info.resolution
        self.map_info.origin_x = msg.info.origin.position.x
        self.map_info.origin_y = msg.info.origin.position.y
        self.map_info.width = msg.info.width
        self.map_info.height = msg.info.height

        if self.exploration_start_time is None:
            self.exploration_start_time = time.time()

        # Update coverage
        total = self.map_info.data.size
        known = np.sum(self.map_info.data != -1)
        self.stats.coverage = (known / total) * 100

    # ==================== Timeout and Collision ====================

    def _check_timeout(self):
        """Check for navigation timeout."""
        if not self.nav_state.is_navigating or not self.nav_state.goal_start_time:
            return

        elapsed = time.time() - self.nav_state.goal_start_time

        if elapsed > Config.EXPLORATION_TIMEOUT:
            self.get_logger().warn(f'Goal timeout ({elapsed:.0f}s), cancelling...')
            self._cancel_goal()
            return

        if elapsed > Config.PROGRESS_CHECK_TIME and self.nav_state.nav_start_pos:
            dist_moved = utils.euclidean_distance(
                self.robot.x, self.robot.y,
                self.nav_state.nav_start_pos[0], self.nav_state.nav_start_pos[1]
            )
            if dist_moved < Config.MIN_PROGRESS_DISTANCE:
                self.get_logger().warn(
                    f'No progress ({dist_moved:.2f}m in {elapsed:.0f}s), cancelling...'
                )
                self._cancel_goal()
                return
            self.nav_state.nav_start_pos = (self.robot.x, self.robot.y)

    def _check_collision(self):
        """Check for imminent collision."""
        if self.collision_handler.check_and_handle_collision(
            self.nav_state.is_navigating
        ):
            # Collision handled - but DON'T cancel goal immediately
            # Let Nav2 re-plan path to the same goal
            # Only cancel after multiple collision events
            self.nav_state.collision_count += 1

            if self.nav_state.collision_count >= 3:
                # Too many collisions to same goal, give up
                self.get_logger().warn(
                    f'Multiple collisions ({self.nav_state.collision_count}), cancelling goal'
                )
                self._mark_current_goal_failed(blocked=True)
                if self.nav_state.goal_handle:
                    self.nav_state.goal_handle.cancel_goal_async()
                self.nav_state.is_navigating = False
                self.nav_state.collision_count = 0
                self.stats.consecutive_failures += 1
                self.current_distance_stage = min(
                    self.current_distance_stage + 1,
                    len(Config.DISTANCE_STAGES) - 1
                )
            else:
                self.get_logger().info(
                    f'Collision avoided ({self.nav_state.collision_count}/3), continuing to goal'
                )

    def _cancel_goal(self):
        """Cancel current navigation goal."""
        if self.nav_state.goal_handle:
            self.nav_state.goal_handle.cancel_goal_async()
        self.nav_state.is_navigating = False
        self.stats.consecutive_failures += 1
        self._mark_current_goal_failed()

    def _mark_current_goal_failed(self, blocked: bool = False):
        """Mark current goal as failed."""
        if self.nav_state.current_goal:
            gx, gy = utils.world_to_grid(
                self.nav_state.current_goal[0],
                self.nav_state.current_goal[1],
                self.map_info
            )
            if gx is not None:
                divisor = (Config.BLOCKED_GOAL_GRID_DIVISOR if blocked
                          else Config.FAILED_GOAL_GRID_DIVISOR)
                self.failed_goals.add((gx // divisor, gy // divisor))
                if blocked:
                    self.unreachable_frontiers.add(
                        (gx // Config.UNREACHABLE_GRID_DIVISOR,
                         gy // Config.UNREACHABLE_GRID_DIVISOR)
                    )

    # ==================== Path Validation ====================

    def _check_path_exists(
        self,
        goal_x: float,
        goal_y: float,
        timeout: float = Config.PATH_CHECK_TIMEOUT
    ) -> Tuple[bool, Optional[float]]:
        """Check if path to goal exists and return path length."""
        if not self.compute_path_client.wait_for_server(timeout_sec=1.0):
            return True, None

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = goal_x
        goal_msg.goal.pose.position.y = goal_y
        goal_msg.goal.pose.orientation.w = 1.0

        goal_msg.start.header.frame_id = 'map'
        goal_msg.start.header.stamp = self.get_clock().now().to_msg()
        goal_msg.start.pose.position.x = self.robot.x
        goal_msg.start.pose.position.y = self.robot.y
        goal_msg.start.pose.orientation.w = 1.0

        future = self.compute_path_client.send_goal_async(goal_msg)

        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                return True, None
            time.sleep(0.05)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return False, None

        result_future = goal_handle.get_result_async()

        while not result_future.done():
            if time.time() - start_time > timeout:
                return True, None
            time.sleep(0.05)

        result = result_future.result()
        if result.result.path.poses:
            path_length = utils.calculate_path_length(result.result.path.poses)
            return True, path_length
        return False, None

    # ==================== Frontier Selection ====================

    def _select_best_frontier(self, clusters: list) -> Optional[dict]:
        """Select best frontier using dynamic distance staging."""
        if not clusters:
            return None

        for stage_idx in range(self.current_distance_stage, len(Config.DISTANCE_STAGES)):
            max_dist = Config.DISTANCE_STAGES[stage_idx]
            min_dist = Config.MIN_GOAL_DISTANCE

            best = self._find_frontier_in_range(clusters, min_dist, max_dist)
            if best:
                self.get_logger().info(
                    f'Found frontier at stage {stage_idx} (max {max_dist}m)'
                )
                self.current_distance_stage = max(0, stage_idx - 1)
                return best

        self.current_distance_stage = 0
        return None

    def _find_frontier_in_range(
        self,
        clusters: list,
        min_dist: float,
        max_dist: float
    ) -> Optional[dict]:
        """Find best frontier within distance range."""
        best = None
        best_score = -1

        for cluster in clusters:
            cx, cy = cluster.center_grid

            # Find valid navigation point
            vx, vy = self.frontier_detector.find_nearest_valid_point(
                cx, cy, self.map_info, self.failed_goals
            )
            if vx is None:
                continue

            wx, wy = utils.grid_to_world(vx, vy, self.map_info)
            if wx is None:
                continue

            euclidean_dist = utils.euclidean_distance(wx, wy, self.robot.x, self.robot.y)

            # Distance filter
            if euclidean_dist < min_dist or euclidean_dist > max_dist:
                continue

            # Unreachable filter
            frontier_key = (vx // Config.UNREACHABLE_GRID_DIVISOR,
                           vy // Config.UNREACHABLE_GRID_DIVISOR)
            if frontier_key in self.unreachable_frontiers:
                continue

            # Check path
            path_exists, path_length = self._check_path_exists(wx, wy)
            if not path_exists:
                self.unreachable_frontiers.add(frontier_key)
                self._limit_set_size(self.unreachable_frontiers,
                                    Config.MAX_UNREACHABLE_FRONTIERS)
                continue

            # Score frontier (with direction consistency)
            frontier = self.frontier_scorer.score_frontier(
                cluster, self.map_info, self.robot, (vx, vy), path_length,
                last_direction=self.last_goal_direction
            )

            if frontier.score > best_score:
                best_score = frontier.score
                best = {
                    'world': frontier.world_pos,
                    'grid': frontier.grid_pos,
                    'euclidean_dist': frontier.euclidean_dist,
                    'path_dist': frontier.path_dist,
                    'size': frontier.size,
                    'info_gain': frontier.info_gain,
                    'openness': frontier.openness,
                    'score': frontier.score
                }

        return best

    def _get_random_free_goal(self) -> Optional[Tuple[float, float]]:
        """Get a random free cell as fallback goal."""
        if not self.map_info.is_valid():
            return None

        free_cells = np.argwhere(self.map_info.data == 0)
        if len(free_cells) == 0:
            return None

        robot_gx, robot_gy = utils.world_to_grid(
            self.robot.x, self.robot.y, self.map_info
        )
        if robot_gx is None:
            return None

        distances = np.sqrt(
            (free_cells[:, 1] - robot_gx) ** 2 +
            (free_cells[:, 0] - robot_gy) ** 2
        )

        valid_mask = (
            (distances > Config.RANDOM_GOAL_MIN_DIST) &
            (distances < Config.RANDOM_GOAL_MAX_DIST)
        )
        valid_cells = free_cells[valid_mask]

        if len(valid_cells) == 0:
            return None

        idx = np.random.randint(len(valid_cells))
        gy, gx = valid_cells[idx]

        # Validate
        if self.frontier_detector._is_goal_valid(gx, gy, self.map_info, self.failed_goals):
            return utils.grid_to_world(gx, gy, self.map_info)

        return None

    # ==================== Navigation ====================

    def _send_goal(self, x: float, y: float):
        """Send single navigation goal."""
        # Repetition check
        target_key = (round(x, 1), round(y, 1))
        if target_key == self.last_target_key:
            self.same_target_count += 1
            if self.same_target_count >= Config.SAME_TARGET_MAX_COUNT:
                self.get_logger().warn(
                    f'Same target {target_key} repeated {self.same_target_count} times, blocking!'
                )
                gx, gy = utils.world_to_grid(x, y, self.map_info)
                if gx is not None:
                    self.failed_goals.add(
                        (gx // Config.BLOCKED_GOAL_GRID_DIVISOR,
                         gy // Config.BLOCKED_GOAL_GRID_DIVISOR)
                    )
                    self.unreachable_frontiers.add(
                        (gx // Config.UNREACHABLE_GRID_DIVISOR,
                         gy // Config.UNREACHABLE_GRID_DIVISOR)
                    )
                self.same_target_count = 0
                self.last_target_key = None
                return
        else:
            self.last_target_key = target_key
            self.same_target_count = 1

        self._start_navigation(x, y)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        angle = math.atan2(y - self.robot.y, x - self.robot.x)
        goal_msg.pose.pose.orientation.z = math.sin(angle / 2)
        goal_msg.pose.pose.orientation.w = math.cos(angle / 2)

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _send_waypoints(self, waypoints: List[Tuple[float, float]]):
        """Send multiple waypoints."""
        if len(waypoints) == 1:
            self._send_goal(waypoints[0][0], waypoints[0][1])
            return

        if not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self._send_goal(waypoints[0][0], waypoints[0][1])
            return

        self._start_navigation(waypoints[-1][0], waypoints[-1][1])

        goal_msg = NavigateThroughPoses.Goal()

        for wx, wy in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wx
            pose.pose.position.y = wy

            angle = math.atan2(wy - self.robot.y, wx - self.robot.x)
            pose.pose.orientation.z = math.sin(angle / 2)
            pose.pose.orientation.w = math.cos(angle / 2)

            goal_msg.poses.append(pose)

        self.get_logger().info(f'Sending {len(waypoints)} waypoints')
        future = self.nav_through_poses_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _start_navigation(self, x: float, y: float):
        """Initialize navigation state."""
        self.nav_state.is_navigating = True
        self.nav_state.current_goal = (x, y)
        self.nav_state.goal_start_time = time.time()
        self.nav_state.nav_start_pos = (self.robot.x, self.robot.y)
        self.nav_state.collision_count = 0  # Reset collision count for new goal
        self.stats.total_goals += 1
        self.last_goal_direction = math.atan2(y - self.robot.y, x - self.robot.x)
        self._publish_goal_marker(x, y)

    def _publish_goal_marker(self, x: float, y: float):
        """Publish goal position marker for RViz visualization."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'exploration_goal'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime.sec = 30
        self.goal_marker_pub.publish(marker)

    def _get_waypoints(self, goal_x: float, goal_y: float) -> List[Tuple[float, float]]:
        """Generate waypoints for long paths."""
        dist = utils.euclidean_distance(goal_x, goal_y, self.robot.x, self.robot.y)

        if dist <= Config.WAYPOINT_INTERVAL * 1.5:
            return [(goal_x, goal_y)]

        num_waypoints = int(dist / Config.WAYPOINT_INTERVAL)
        waypoints = []

        for i in range(1, num_waypoints + 1):
            ratio = i / (num_waypoints + 1)
            wx = self.robot.x + ratio * (goal_x - self.robot.x)
            wy = self.robot.y + ratio * (goal_y - self.robot.y)
            waypoints.append((wx, wy))

        waypoints.append((goal_x, goal_y))
        return waypoints

    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        self.nav_state.goal_handle = future.result()

        if not self.nav_state.goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.nav_state.is_navigating = False
            self.stats.consecutive_failures += 1
            return

        result_future = self.nav_state.goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached!')
            self.stats.consecutive_failures = 0
            self.stats.successful_goals += 1
            self.current_distance_stage = max(0, self.current_distance_stage - 1)
        else:
            self.get_logger().warn(f'Goal failed (status: {status})')
            self.stats.consecutive_failures += 1
            self._mark_current_goal_failed()

        self.nav_state.is_navigating = False

    # ==================== Main Loop ====================

    def _explore_callback(self):
        """Main exploration loop."""
        if self.nav_state.is_navigating:
            return

        if not self.map_info.is_valid():
            self.get_logger().info('Waiting for map...')
            return

        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            return

        # Log progress
        elapsed = time.time() - (self.exploration_start_time or time.time())
        self.get_logger().info(
            f'Coverage: {self.stats.coverage:.1f}% | '
            f'Goals: {self.stats.successful_goals}/{self.stats.total_goals} '
            f'({self.stats.success_rate:.0f}%) | '
            f'Time: {elapsed:.0f}s'
        )

        if self.stats.coverage > 95:
            self.get_logger().info('Exploration complete!')
            return

        # Check oscillation
        if self.recovery_manager.detect_oscillation(min_goals=self.stats.total_goals):
            self.recovery_manager.handle_oscillation()
            self._reset_exploration_state()
            return

        # Check if stuck
        current_pos = (round(self.robot.x, 1), round(self.robot.y, 1))
        if current_pos == self.last_robot_pos:
            self.stats.stuck_count += 1
        else:
            self.stats.stuck_count = 0
        self.last_robot_pos = current_pos

        # Recovery if stuck
        if self.stats.consecutive_failures >= 3 or self.stats.stuck_count >= 5:
            self.get_logger().warn('Stuck detected! Recovery mode...')
            self.recovery_manager.backup()
            self.recovery_manager.full_recovery()
            self._reset_exploration_state()
            return

        if self.stats.consecutive_failures >= 1:
            self.recovery_manager.clear_costmaps()
            self.current_distance_stage = min(
                self.current_distance_stage + 1,
                len(Config.DISTANCE_STAGES) - 1
            )

        # Find frontiers
        frontier_points = self.frontier_detector.find_frontiers(self.map_info)
        clusters = self.frontier_detector.cluster_frontiers(
            frontier_points,
            (self.map_info.height, self.map_info.width)
        )

        goal = None
        path_dist = None

        if clusters:
            best = self._select_best_frontier(clusters)
            if best:
                goal = best['world']
                path_dist = best.get('path_dist', best['euclidean_dist'])
                self.get_logger().info(
                    f'Target: ({goal[0]:.2f}, {goal[1]:.2f}) '
                    f'dist={best["euclidean_dist"]:.1f}m path={path_dist:.1f}m '
                    f'size={best["size"]} gain={best["info_gain"]:.0f}'
                )

        if goal is None:
            self.get_logger().info('No frontier found, trying random goal...')
            result = self._get_random_free_goal()
            if result:
                goal = result
                path_dist = utils.euclidean_distance(
                    goal[0], goal[1], self.robot.x, self.robot.y
                )

        if goal:
            if path_dist and path_dist > Config.WAYPOINT_INTERVAL * 2:
                waypoints = self._get_waypoints(goal[0], goal[1])
                if len(waypoints) > 1:
                    self.get_logger().info(f'Using {len(waypoints)} waypoints')
                self._send_waypoints(waypoints)
            else:
                self._send_goal(goal[0], goal[1])
        else:
            self.get_logger().warn('No valid goal found')
            self.recovery_manager.full_recovery()

    def _reset_exploration_state(self):
        """Reset exploration state after recovery."""
        self.failed_goals.clear()
        self.unreachable_frontiers.clear()
        self.stats.consecutive_failures = 0
        self.stats.stuck_count = 0
        self.current_distance_stage = 0

    def _limit_set_size(self, s: set, max_size: int):
        """Limit set size to prevent memory leak."""
        while len(s) > max_size:
            s.pop()


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'\n=== Final Stats ===\n'
            f'Coverage: {node.stats.coverage:.1f}%\n'
            f'Goals: {node.stats.successful_goals}/{node.stats.total_goals} '
            f'({node.stats.success_rate:.0f}%)'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
