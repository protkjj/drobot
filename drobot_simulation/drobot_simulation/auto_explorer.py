#!/usr/bin/env python3
"""
Auto Explorer Node - Improved Version
자동으로 미탐색 영역(frontier)을 찾아 탐색하는 노드

개선 사항:
- 로봇 현재 위치 기반 frontier 선택
- 거리 + 크기 + 참신함을 고려한 점수 시스템
- Stuck 감지 및 복구
- 더 나은 탐색 영역 기억
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import numpy as np
import random
import math


class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')

        # Parameters
        self.declare_parameter('min_frontier_size', 3)  # 더 작은 frontier도 탐색
        self.declare_parameter('exploration_timeout', 60.0)
        self.declare_parameter('goal_tolerance', 0.5)

        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.exploration_timeout = self.get_parameter('exploration_timeout').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # Map subscription
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )

        # Odometry subscription (로봇 현재 위치)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # State
        self.current_map = None
        self.map_info = None
        self.is_exploring = False
        self.goal_handle = None

        # 로봇 현재 위치
        self.robot_x = 0.0
        self.robot_y = 0.0

        # 탐색 기록 (더 많이 저장)
        self.explored_goals = []  # [(x, y, timestamp), ...]
        self.failed_goals = []    # 실패한 목표들 (블랙리스트)
        self.consecutive_failures = 0  # 연속 실패 횟수

        # 탐색 모드
        self.exploration_mode = 'normal'  # 'normal', 'recovery', 'random'

        # Timer for exploration
        self.explore_timer = self.create_timer(2.0, self.explore_callback)  # 더 자주 체크

        self.get_logger().info('Improved Auto Explorer initialized!')

    def odom_callback(self, msg):
        """로봇 위치 업데이트"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def map_callback(self, msg):
        """지도 업데이트 콜백"""
        self.current_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def find_frontiers(self):
        """미탐색 영역(frontier) 찾기 - 최적화 버전"""
        if self.current_map is None:
            return []

        frontiers = []
        height, width = self.current_map.shape

        # Frontier: 탐색된 영역(0)과 미탐색 영역(-1)의 경계
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if self.current_map[y, x] == 0:  # 자유 공간
                    # 주변에 미탐색 영역(-1)이 있는지 확인
                    neighbors = self.current_map[y-1:y+2, x-1:x+2].flatten()
                    if -1 in neighbors:
                        # 장애물 근처는 제외 (안전한 frontier만)
                        if 100 not in neighbors:
                            frontiers.append((x, y))

        return frontiers

    def cluster_frontiers(self, frontiers):
        """Frontier들을 클러스터링"""
        if not frontiers:
            return []

        frontier_set = set(frontiers)
        clusters = []
        visited = set()

        for fx, fy in frontiers:
            if (fx, fy) in visited:
                continue

            # BFS로 인접한 frontier들 그룹화
            cluster = []
            queue = [(fx, fy)]

            while queue:
                x, y = queue.pop(0)
                if (x, y) in visited:
                    continue
                visited.add((x, y))
                cluster.append((x, y))

                # 인접한 frontier 찾기 (더 넓은 범위)
                for dx in [-2, -1, 0, 1, 2]:
                    for dy in [-2, -1, 0, 1, 2]:
                        nx, ny = x + dx, y + dy
                        if (nx, ny) in frontier_set and (nx, ny) not in visited:
                            queue.append((nx, ny))

            if len(cluster) >= self.min_frontier_size:
                # 클러스터의 가장 안전한 점 찾기 (중심에서 가장 가까운 자유 공간)
                cx = sum(p[0] for p in cluster) // len(cluster)
                cy = sum(p[1] for p in cluster) // len(cluster)

                # 중심점이 안전한지 확인, 아니면 클러스터 내 다른 점 사용
                safe_point = self.find_safe_goal_point(cx, cy, cluster)
                if safe_point:
                    clusters.append((safe_point[0], safe_point[1], len(cluster)))

        return clusters

    def find_safe_goal_point(self, cx, cy, cluster):
        """목표점 주변이 안전한지 확인하고 안전한 점 반환"""
        if self.current_map is None:
            return (cx, cy)

        height, width = self.current_map.shape

        # 중심점 주변 검사
        for radius in range(0, 10):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        # 자유 공간이고 장애물에서 충분히 떨어진 곳
                        if self.current_map[ny, nx] == 0:
                            if self.is_safe_position(nx, ny):
                                return (nx, ny)

        return None

    def is_safe_position(self, gx, gy, safety_margin=3):
        """해당 위치가 안전한지 확인 (장애물에서 충분히 떨어져 있는지)"""
        if self.current_map is None:
            return False

        height, width = self.current_map.shape

        for dx in range(-safety_margin, safety_margin + 1):
            for dy in range(-safety_margin, safety_margin + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    if self.current_map[ny, nx] == 100:  # 장애물
                        return False
        return True

    def grid_to_world(self, gx, gy):
        """그리드 좌표를 월드 좌표로 변환"""
        if self.map_info is None:
            return None, None

        wx = self.map_info.origin.position.x + (gx + 0.5) * self.map_info.resolution
        wy = self.map_info.origin.position.y + (gy + 0.5) * self.map_info.resolution
        return wx, wy

    def world_to_grid(self, wx, wy):
        """월드 좌표를 그리드 좌표로 변환"""
        if self.map_info is None:
            return None, None

        gx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        return gx, gy

    def calculate_frontier_score(self, wx, wy, size):
        """Frontier 점수 계산 (높을수록 좋음)"""
        # 1. 로봇과의 거리 (너무 가깝지도, 멀지도 않은 게 좋음)
        dist_to_robot = math.sqrt((wx - self.robot_x)**2 + (wy - self.robot_y)**2)

        # 최소 거리 임계값: 0.5m 미만은 무시 (goal_tolerance보다 작으면 의미 없음)
        if dist_to_robot < 0.5:
            return -1  # 너무 가까운 frontier는 제외

        # 이상적인 거리: 1.5~5m
        if dist_to_robot < 1.0:
            distance_score = 0.5  # 약간 가까움
        elif dist_to_robot < 1.5:
            distance_score = 0.8
        elif dist_to_robot < 5.0:
            distance_score = 1.0  # 이상적
        elif dist_to_robot < 10.0:
            distance_score = 0.8
        else:
            distance_score = 0.5  # 너무 멀음

        # 2. 탐색 기록과의 거리 (이미 가본 곳에서 멀수록 좋음)
        min_dist_to_explored = float('inf')
        for ex, ey, _ in self.explored_goals[-30:]:  # 최근 30개
            dist = math.sqrt((wx - ex)**2 + (wy - ey)**2)
            min_dist_to_explored = min(min_dist_to_explored, dist)

        if min_dist_to_explored == float('inf'):
            novelty_score = 1.0
        elif min_dist_to_explored < 0.8:
            return -1  # 이미 가본 곳 근처는 완전히 제외
        elif min_dist_to_explored < 1.5:
            novelty_score = 0.3  # 낮은 점수
        elif min_dist_to_explored < 2.5:
            novelty_score = 0.6
        else:
            novelty_score = 1.0

        # 3. 실패한 목표 근처인지 (블랙리스트)
        for fx, fy in self.failed_goals[-20:]:
            dist = math.sqrt((wx - fx)**2 + (wy - fy)**2)
            if dist < 1.5:
                return -1  # 블랙리스트 근처는 제외

        # 4. 크기 점수 (크면 좋지만 너무 치우치지 않게)
        size_score = min(1.0, size / 20.0)

        # 최종 점수 (참신함에 가중치를 더 줌)
        total_score = (distance_score * 0.3 +
                      novelty_score * 0.5 +
                      size_score * 0.2)

        return total_score

    def select_best_frontier(self, clusters):
        """가장 좋은 frontier 선택"""
        if not clusters:
            return None

        best_cluster = None
        best_score = -float('inf')

        scored_clusters = []

        for cx, cy, size in clusters:
            wx, wy = self.grid_to_world(cx, cy)
            if wx is None:
                continue

            score = self.calculate_frontier_score(wx, wy, size)

            # 음수 점수는 무효한 frontier (너무 가깝거나 이미 방문한 곳)
            if score < 0:
                continue

            scored_clusters.append((cx, cy, size, score, wx, wy))

            if score > best_score:
                best_score = score
                best_cluster = (cx, cy, size)

        # 디버그 로그
        if scored_clusters:
            scored_clusters.sort(key=lambda x: x[3], reverse=True)
            top_3 = scored_clusters[:3]
            self.get_logger().info(f'Top frontiers: ' +
                ', '.join([f'({c[4]:.1f},{c[5]:.1f})={c[3]:.2f}' for c in top_3]))
        else:
            self.get_logger().info('No valid frontiers (all filtered out)')

        return best_cluster

    def explore_callback(self):
        """주기적으로 탐색 수행"""
        if self.is_exploring:
            return

        if self.current_map is None:
            self.get_logger().info('Waiting for map data...')
            return

        # Wait for Nav2
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 not available yet...')
            return

        # Recovery 모드 체크
        if self.consecutive_failures >= 3:
            self.get_logger().warn(f'Multiple failures ({self.consecutive_failures}), trying recovery...')
            self.exploration_mode = 'recovery'
            self.consecutive_failures = 0

        # Find frontiers
        frontiers = self.find_frontiers()
        clusters = self.cluster_frontiers(frontiers)

        if not clusters:
            self.get_logger().info('No frontiers found! Map exploration complete!')
            return

        self.get_logger().info(f'Found {len(clusters)} frontier clusters (mode: {self.exploration_mode})')

        # Frontier 선택
        if self.exploration_mode == 'recovery':
            # Recovery 모드: 랜덤하게 먼 곳 선택
            random.shuffle(clusters)
            goal_cluster = clusters[0] if clusters else None
            self.exploration_mode = 'normal'
        else:
            # Normal 모드: 점수 기반 선택
            goal_cluster = self.select_best_frontier(clusters)

        # 모든 frontier가 필터링되면 랜덤 방향으로 탐색
        if goal_cluster is None:
            self.get_logger().info('No valid frontiers, trying random exploration...')
            # 랜덤 방향으로 2~4m 이동
            angle = random.uniform(0, 2 * math.pi)
            distance = random.uniform(2.0, 4.0)
            rx = self.robot_x + distance * math.cos(angle)
            ry = self.robot_y + distance * math.sin(angle)
            self.send_goal(rx, ry)
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.explored_goals.append((rx, ry, current_time))
            return

        if goal_cluster:
            gx, gy, size = goal_cluster
            wx, wy = self.grid_to_world(gx, gy)

            if wx is not None:
                self.send_goal(wx, wy)
                # 탐색 기록에 추가
                current_time = self.get_clock().now().nanoseconds / 1e9
                self.explored_goals.append((wx, wy, current_time))

                # 오래된 기록 정리 (50개 초과 시)
                if len(self.explored_goals) > 50:
                    self.explored_goals = self.explored_goals[-50:]

    def send_goal(self, x, y):
        """Nav2에 목표점 전송"""
        self.is_exploring = True
        self.current_goal = (x, y)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending goal: ({x:.2f}, {y:.2f}) [robot at ({self.robot_x:.2f}, {self.robot_y:.2f})]')

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """목표 전송 응답 콜백"""
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.is_exploring = False
            self.consecutive_failures += 1
            if hasattr(self, 'current_goal'):
                self.failed_goals.append(self.current_goal)
            return

        self.get_logger().info('Goal accepted')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """목표 도달 결과 콜백"""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached! Looking for next frontier...')
            self.consecutive_failures = 0  # 성공하면 리셋
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')
            self.consecutive_failures += 1
            if hasattr(self, 'current_goal'):
                self.failed_goals.append(self.current_goal)
                # 실패 목록 정리
                if len(self.failed_goals) > 30:
                    self.failed_goals = self.failed_goals[-30:]

        self.is_exploring = False

    def feedback_callback(self, feedback_msg):
        """네비게이션 피드백"""
        # 진행 상황 모니터링 가능
        pass


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
