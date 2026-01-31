#!/usr/bin/env python3
"""
Auto Explorer Node - Optimized Version v2
빠르고 효율적인 Frontier 기반 탐색

주요 개선:
- NumPy 기반 빠른 frontier 탐지
- 가장 가까운 frontier 우선 탐색 (Greedy)
- Costmap 클리어 기능 추가 (Start occupied 해결)
- 적극적인 recovery 전략
- 빠른 탐색 주기
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from action_msgs.msg import GoalStatus
from scipy import ndimage
import numpy as np
import math
from collections import deque
import time


class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')

        # Parameters - 더 공격적인 탐색
        self.declare_parameter('min_frontier_size', 3)
        self.declare_parameter('exploration_timeout', 15.0)  # 목표당 최대 시간 (30→15초)
        self.declare_parameter('safety_margin', 2)  # 줄임: 4 → 2
        self.declare_parameter('max_goal_distance', 5.0)  # 줄임: 8 → 5
        self.declare_parameter('min_goal_distance', 0.4)  # 줄임: 0.8 → 0.4

        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.exploration_timeout = self.get_parameter('exploration_timeout').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.max_goal_distance = self.get_parameter('max_goal_distance').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value

        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Cmd_vel for recovery rotation
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Costmap clear services
        self.clear_global_costmap = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'
        )
        self.clear_local_costmap = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'
        )

        # State
        self.current_map = None
        self.map_info = None
        self.is_navigating = False
        self.goal_handle = None
        self.current_goal = None
        self.goal_start_time = None

        # Robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Exploration tracking
        self.failed_goals = set()  # (gx, gy) 그리드 좌표
        self.visited_cells = set()
        self.consecutive_failures = 0
        self.total_goals = 0
        self.successful_goals = 0

        # Progress tracking
        self.last_coverage = 0.0
        self.exploration_start_time = None
        self.last_robot_pos = None
        self.stuck_count = 0

        # Timer - 더 빠른 주기
        self.explore_timer = self.create_timer(1.0, self.explore_callback)  # 2.5 → 1.0초
        self.timeout_timer = self.create_timer(1.0, self.check_timeout)

        self.get_logger().info('Optimized Auto Explorer started!')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z))

    def map_callback(self, msg):
        self.current_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

        if self.exploration_start_time is None:
            self.exploration_start_time = time.time()

        # Coverage 계산
        total = self.current_map.size
        known = np.sum(self.current_map != -1)
        self.last_coverage = (known / total) * 100

    def check_timeout(self):
        """목표 타임아웃 및 진행 체크"""
        if not self.is_navigating or not self.goal_start_time:
            return

        elapsed = time.time() - self.goal_start_time

        # 시간 타임아웃
        if elapsed > self.exploration_timeout:
            self.get_logger().warn(f'Goal timeout ({elapsed:.0f}s), cancelling...')
            self.cancel_goal()
            return

        # 진행 체크 - 5초마다 위치 변화 확인
        if elapsed > 5.0 and hasattr(self, 'nav_start_pos'):
            dist_moved = math.sqrt(
                (self.robot_x - self.nav_start_pos[0])**2 +
                (self.robot_y - self.nav_start_pos[1])**2
            )
            # 5초 동안 0.3m 미만 이동 = stuck
            if dist_moved < 0.3:
                self.get_logger().warn(f'No progress ({dist_moved:.2f}m in {elapsed:.0f}s), cancelling...')
                self.cancel_goal()
                return
            # 진행 중이면 시작 위치 업데이트
            self.nav_start_pos = (self.robot_x, self.robot_y)

    def cancel_goal(self):
        """현재 목표 취소"""
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
        self.is_navigating = False
        self.consecutive_failures += 1
        if self.current_goal:
            gx, gy = self.world_to_grid(self.current_goal[0], self.current_goal[1])
            if gx is not None:
                self.failed_goals.add((gx // 5, gy // 5))  # 5셀 단위로 블랙리스트

    def find_frontiers_fast(self):
        """NumPy 기반 빠른 frontier 탐지"""
        if self.current_map is None:
            return []

        # 자유 공간과 미탐색 영역 마스크
        free = (self.current_map == 0)
        unknown = (self.current_map == -1)

        # 미탐색 영역 팽창 (인접 셀 찾기)
        unknown_dilated = ndimage.binary_dilation(unknown, iterations=1)

        # Frontier = 자유 공간 AND 미탐색 영역 인접
        frontier_mask = free & unknown_dilated

        # 장애물 근처 제외
        occupied = (self.current_map == 100)
        obstacle_nearby = ndimage.binary_dilation(occupied, iterations=self.safety_margin)
        frontier_mask = frontier_mask & ~obstacle_nearby

        # Frontier 좌표 추출
        frontier_points = np.argwhere(frontier_mask)

        return frontier_points  # (y, x) 형태

    def cluster_frontiers(self, frontier_points):
        """Frontier 클러스터링 - 연결된 영역 찾기"""
        if len(frontier_points) == 0:
            return []

        # 라벨링으로 연결 영역 찾기
        frontier_mask = np.zeros_like(self.current_map, dtype=bool)
        for y, x in frontier_points:
            frontier_mask[y, x] = True

        labeled, num_features = ndimage.label(frontier_mask)

        clusters = []
        for i in range(1, num_features + 1):
            points = np.argwhere(labeled == i)
            if len(points) >= self.min_frontier_size:
                # 클러스터 중심
                cy, cx = points.mean(axis=0).astype(int)
                clusters.append({
                    'center_grid': (cx, cy),
                    'size': len(points),
                    'points': points
                })

        return clusters

    def grid_to_world(self, gx, gy):
        if self.map_info is None:
            return None, None
        wx = self.map_info.origin.position.x + (gx + 0.5) * self.map_info.resolution
        wy = self.map_info.origin.position.y + (gy + 0.5) * self.map_info.resolution
        return wx, wy

    def world_to_grid(self, wx, wy):
        if self.map_info is None:
            return None, None
        gx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        return gx, gy

    def is_goal_valid(self, gx, gy):
        """목표가 유효한지 확인"""
        if self.current_map is None or self.map_info is None:
            return False

        h, w = self.current_map.shape
        if not (0 <= gx < w and 0 <= gy < h):
            return False

        # 자유 공간인지
        if self.current_map[gy, gx] != 0:
            return False

        # 블랙리스트 체크
        if (gx // 5, gy // 5) in self.failed_goals:
            return False

        # 장애물과 거리 체크
        margin = self.safety_margin
        y_min, y_max = max(0, gy - margin), min(h, gy + margin + 1)
        x_min, x_max = max(0, gx - margin), min(w, gx + margin + 1)
        region = self.current_map[y_min:y_max, x_min:x_max]
        if np.any(region == 100):
            return False

        return True

    def find_nearest_valid_point(self, gx, gy, max_search=20):
        """가장 가까운 유효한 점 찾기 (나선형 탐색)"""
        for r in range(max_search):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) == r or abs(dy) == r:  # 테두리만
                        nx, ny = gx + dx, gy + dy
                        if self.is_goal_valid(nx, ny):
                            return nx, ny
        return None, None

    def select_best_frontier(self, clusters):
        """가장 좋은 frontier 선택 - 거리 기반 greedy"""
        if not clusters:
            return None

        robot_gx, robot_gy = self.world_to_grid(self.robot_x, self.robot_y)
        if robot_gx is None:
            return None

        best = None
        best_score = -1

        for cluster in clusters:
            cx, cy = cluster['center_grid']

            # 유효한 점 찾기
            vx, vy = self.find_nearest_valid_point(cx, cy)
            if vx is None:
                continue

            wx, wy = self.grid_to_world(vx, vy)
            if wx is None:
                continue

            dist = math.sqrt((wx - self.robot_x)**2 + (wy - self.robot_y)**2)

            # 거리 필터
            if dist < self.min_goal_distance or dist > self.max_goal_distance:
                continue

            # 점수: 가까울수록 + 크기 클수록 좋음
            # 가까운 거리 우선 (greedy)
            dist_score = 1.0 / (dist + 0.1)
            size_score = min(cluster['size'] / 20.0, 1.0)

            # 방향 보너스 (현재 방향과 일치하면)
            goal_angle = math.atan2(wy - self.robot_y, wx - self.robot_x)
            angle_diff = abs(self.normalize_angle(goal_angle - self.robot_yaw))
            dir_score = 1.0 - (angle_diff / math.pi)

            score = dist_score * 0.5 + size_score * 0.2 + dir_score * 0.3

            if score > best_score:
                best_score = score
                best = {
                    'world': (wx, wy),
                    'grid': (vx, vy),
                    'dist': dist,
                    'size': cluster['size'],
                    'score': score
                }

        return best

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def do_backup_move(self):
        """약간 후진하여 occupied 상태 탈출"""
        self.get_logger().info('Recovery: Backing up...')
        twist = Twist()
        twist.linear.x = -0.1  # 후진
        for _ in range(10):
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)

    def clear_costmaps(self):
        """Costmap 클리어 - Start occupied 문제 해결"""
        self.get_logger().info('Clearing costmaps...')

        # Global costmap clear
        if self.clear_global_costmap.wait_for_service(timeout_sec=1.0):
            req = ClearEntireCostmap.Request()
            self.clear_global_costmap.call_async(req)

        # Local costmap clear
        if self.clear_local_costmap.wait_for_service(timeout_sec=1.0):
            req = ClearEntireCostmap.Request()
            self.clear_local_costmap.call_async(req)

        time.sleep(0.5)  # costmap 업데이트 대기

    def do_recovery_rotation(self):
        """제자리 회전으로 주변 스캔 + costmap 클리어"""
        self.get_logger().info('Recovery: Clearing costmaps and rotating...')

        # 먼저 costmap 클리어
        self.clear_costmaps()

        # 회전
        twist = Twist()
        twist.angular.z = 0.5
        for _ in range(15):  # 더 오래 회전
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        time.sleep(0.3)

        # 회전 후 다시 costmap 클리어
        self.clear_costmaps()

    def get_random_free_goal(self):
        """랜덤 자유 공간 목표"""
        if self.current_map is None:
            return None

        free_cells = np.argwhere(self.current_map == 0)
        if len(free_cells) == 0:
            return None

        # 로봇에서 적당한 거리의 랜덤 점
        robot_gx, robot_gy = self.world_to_grid(self.robot_x, self.robot_y)
        if robot_gx is None:
            return None

        # 거리 계산
        distances = np.sqrt((free_cells[:, 1] - robot_gx)**2 +
                           (free_cells[:, 0] - robot_gy)**2)

        # 1~4m 거리의 셀들
        valid_mask = (distances > 20) & (distances < 80)  # 그리드 셀 단위 (0.05m)
        valid_cells = free_cells[valid_mask]

        if len(valid_cells) == 0:
            return None

        # 랜덤 선택
        idx = np.random.randint(len(valid_cells))
        gy, gx = valid_cells[idx]

        if self.is_goal_valid(gx, gy):
            return self.grid_to_world(gx, gy)

        return None

    def explore_callback(self):
        """탐색 메인 루프"""
        if self.is_navigating:
            return

        if self.current_map is None:
            self.get_logger().info('Waiting for map...')
            return

        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            return

        # 진행 상황 로깅
        elapsed = time.time() - (self.exploration_start_time or time.time())
        self.get_logger().info(
            f'Coverage: {self.last_coverage:.1f}% | '
            f'Goals: {self.successful_goals}/{self.total_goals} | '
            f'Time: {elapsed:.0f}s'
        )

        # 탐색 완료 체크
        if self.last_coverage > 95:
            self.get_logger().info('🎉 Exploration complete!')
            return

        # Stuck 체크
        current_pos = (round(self.robot_x, 1), round(self.robot_y, 1))
        if current_pos == self.last_robot_pos:
            self.stuck_count += 1
        else:
            self.stuck_count = 0
        self.last_robot_pos = current_pos

        # 연속 실패 또는 stuck 시 recovery
        if self.consecutive_failures >= 3 or self.stuck_count >= 5:
            self.get_logger().warn('Stuck detected! Recovery mode...')

            # 먼저 약간 후진 시도 (Start occupied 탈출)
            self.do_backup_move()

            self.do_recovery_rotation()
            self.consecutive_failures = 0
            self.stuck_count = 0
            self.failed_goals.clear()  # 블랙리스트 초기화
            return

        # 최근 실패가 많으면 costmap 클리어 후 진행
        if self.consecutive_failures >= 1:
            self.clear_costmaps()

        # Frontier 탐지
        frontier_points = self.find_frontiers_fast()
        clusters = self.cluster_frontiers(frontier_points)

        goal = None

        if clusters:
            # 최적 frontier 선택
            best = self.select_best_frontier(clusters)
            if best:
                goal = best['world']
                self.get_logger().info(
                    f'Target: ({goal[0]:.2f}, {goal[1]:.2f}) '
                    f'dist={best["dist"]:.1f}m size={best["size"]}'
                )

        if goal is None:
            # Frontier 없으면 랜덤 탐색
            self.get_logger().info('No frontier, trying random goal...')
            result = self.get_random_free_goal()
            if result:
                goal = result

        if goal:
            self.send_goal(goal[0], goal[1])
        else:
            self.get_logger().warn('No valid goal found')
            self.do_recovery_rotation()

    def send_goal(self, x, y):
        """Nav2로 목표 전송"""
        self.is_navigating = True
        self.current_goal = (x, y)
        self.goal_start_time = time.time()
        self.nav_start_pos = (self.robot_x, self.robot_y)  # 진행 체크용
        self.total_goals += 1

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # 목표 방향
        angle = math.atan2(y - self.robot_y, x - self.robot_x)
        goal_msg.pose.pose.orientation.z = math.sin(angle / 2)
        goal_msg.pose.pose.orientation.w = math.cos(angle / 2)

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.is_navigating = False
            self.consecutive_failures += 1
            return

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✓ Goal reached')
            self.consecutive_failures = 0
            self.successful_goals += 1
        else:
            self.get_logger().warn(f'✗ Goal failed (status: {status})')
            self.consecutive_failures += 1
            if self.current_goal:
                gx, gy = self.world_to_grid(self.current_goal[0], self.current_goal[1])
                if gx is not None:
                    self.failed_goals.add((gx // 5, gy // 5))

        self.is_navigating = False


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'\n=== Final Stats ===\n'
            f'Coverage: {node.last_coverage:.1f}%\n'
            f'Goals: {node.successful_goals}/{node.total_goals}'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
