#!/usr/bin/env python3
"""
Auto Explorer Node - Optimized Version v6
빠르고 효율적인 Frontier 기반 탐색 + 개방 공간 우선

v6 개선사항:
1. 경로 검증 - Nav2 computePath로 도달 가능한 frontier만 선택
2. 동적 목표 거리 - 가까운 것부터 시도, 없으면 점점 멀리
3. 중간 웨이포인트 - 긴 경로를 분할하여 안정적 이동
4. A* 거리 사용 - 실제 경로 길이 기반 점수 계산
5. Information Gain - 미탐색 영역 가시성 기반 frontier 우선순위
6. Oscillation 감지 - 왔다갔다 패턴 감지하여 recovery
7. 실시간 충돌 회피 - LiDAR 기반 선제적 장애물 감지 및 회피
8. 개방 공간 우선 - 벽 근처 frontier 페널티, 넓은 공간 선호
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses, ComputePathToPose
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

        # Parameters
        self.declare_parameter('min_frontier_size', 3)
        self.declare_parameter('exploration_timeout', 20.0)
        self.declare_parameter('safety_margin', 2)
        self.declare_parameter('min_goal_distance', 0.4)
        self.declare_parameter('waypoint_interval', 1.5)
        self.declare_parameter('info_gain_radius', 40)  # 그리드 셀 단위 (2m)

        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.exploration_timeout = self.get_parameter('exploration_timeout').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.waypoint_interval = self.get_parameter('waypoint_interval').value
        self.info_gain_radius = self.get_parameter('info_gain_radius').value

        # 동적 목표 거리 단계
        self.distance_stages = [2.0, 4.0, 6.0, 10.0, 15.0]
        self.current_distance_stage = 0

        # 이전 목표 방향 (약한 연속성 보너스용)
        self.last_goal_direction = None

        # 같은 목표 반복 방지
        self.last_target_key = None
        self.same_target_count = 0

        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        # LiDAR subscription for collision avoidance
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # Collision avoidance parameters
        self.last_scan = None
        self.collision_distance = 0.50  # 50cm - emergency stop threshold (여유있게)
        self.warning_distance = 0.7     # 70cm - slow down threshold
        self.collision_check_angle = math.pi / 6  # 30 degrees front cone (더 좁게)
        self.last_collision_time = 0.0  # 쿨다운용
        self.collision_cooldown = 5.0   # 5초 쿨다운 (더 길게)

        # Cmd_vel for recovery
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Nav2 action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.compute_path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

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
        self.failed_goals = set()
        self.unreachable_frontiers = set()
        self.visited_cells = set()
        self.consecutive_failures = 0
        self.total_goals = 0
        self.successful_goals = 0

        # Progress tracking
        self.last_coverage = 0.0
        self.exploration_start_time = None
        self.last_robot_pos = None
        self.stuck_count = 0

        # 개선 6: Oscillation 감지용 위치 히스토리
        self.position_history = deque(maxlen=20)  # 최근 20개 위치 저장 (더 짧게)
        self.oscillation_threshold = 8  # 같은 위치 반복 횟수 (5→8로 완화)
        self.oscillation_detected = False

        # Timer (빠른 반응)
        self.explore_timer = self.create_timer(0.8, self.explore_callback)  # 1.5 → 0.8초
        self.timeout_timer = self.create_timer(0.5, self.check_timeout)     # 1.0 → 0.5초
        # Collision avoidance timer (higher frequency)
        self.collision_timer = self.create_timer(0.1, self.check_collision)

        self.get_logger().info('Auto Explorer v6 started! (+OpenSpacePreference)')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z))

        # 개선 6: 위치 히스토리 업데이트 (0.5m 단위로 더 거칠게)
        current_pos = (round(self.robot_x * 2) / 2, round(self.robot_y * 2) / 2)
        self.position_history.append(current_pos)

    def scan_callback(self, msg):
        """LiDAR 데이터 수신"""
        self.last_scan = msg

    def map_callback(self, msg):
        self.current_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

        if self.exploration_start_time is None:
            self.exploration_start_time = time.time()

        total = self.current_map.size
        known = np.sum(self.current_map != -1)
        self.last_coverage = (known / total) * 100

    def check_timeout(self):
        """목표 타임아웃 및 진행 체크"""
        if not self.is_navigating or not self.goal_start_time:
            return

        elapsed = time.time() - self.goal_start_time

        if elapsed > self.exploration_timeout:
            self.get_logger().warn(f'Goal timeout ({elapsed:.0f}s), cancelling...')
            self.cancel_goal()
            return

        if elapsed > 10.0 and hasattr(self, 'nav_start_pos'):
            dist_moved = math.sqrt(
                (self.robot_x - self.nav_start_pos[0])**2 +
                (self.robot_y - self.nav_start_pos[1])**2
            )
            if dist_moved < 0.5:
                self.get_logger().warn(f'No progress ({dist_moved:.2f}m in {elapsed:.0f}s), cancelling...')
                self.cancel_goal()
                return
            self.nav_start_pos = (self.robot_x, self.robot_y)

    def cancel_goal(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
        self.is_navigating = False
        self.consecutive_failures += 1
        if self.current_goal:
            gx, gy = self.world_to_grid(self.current_goal[0], self.current_goal[1])
            if gx is not None:
                self.failed_goals.add((gx // 5, gy // 5))

    # ==================== 개선 7: 충돌 회피 ====================

    def check_collision(self):
        """실시간 충돌 감지 및 회피 (0.1초마다 실행)"""
        if self.last_scan is None:
            return

        # 쿨다운 체크 (너무 자주 발동 방지)
        current_time = time.time()
        if current_time - self.last_collision_time < self.collision_cooldown:
            return

        # 네비게이션 중일 때만 체크 (정지 상태에서는 불필요)
        if not self.is_navigating:
            return

        # 전방 장애물 거리 확인
        collision_imminent, min_dist, obstacle_direction = self.analyze_front_obstacles()

        if collision_imminent:
            self.last_collision_time = current_time
            self.handle_imminent_collision(min_dist, obstacle_direction)

    def analyze_front_obstacles(self):
        """전방 장애물 분석"""
        if self.last_scan is None:
            return False, float('inf'), 0.0

        ranges = list(self.last_scan.ranges)
        num_ranges = len(ranges)
        angle_min = self.last_scan.angle_min
        angle_increment = self.last_scan.angle_increment

        # 전방 ±60도 범위 인덱스 계산
        front_center = int((0 - angle_min) / angle_increment)  # 0도 = 정면
        half_cone = int(self.collision_check_angle / angle_increment)

        front_start = max(0, front_center - half_cone)
        front_end = min(num_ranges - 1, front_center + half_cone)

        # 전방 범위 데이터 추출
        front_ranges = ranges[front_start:front_end + 1]

        # 유효한 범위만 필터링 (너무 가깝거나 너무 먼 값 제외)
        valid_data = []
        for i, r in enumerate(front_ranges):
            if self.last_scan.range_min < r < self.last_scan.range_max:
                angle = angle_min + (front_start + i) * angle_increment
                valid_data.append((r, angle))

        if not valid_data:
            return False, float('inf'), 0.0

        # 최소 거리와 해당 방향 찾기
        min_dist = float('inf')
        min_angle = 0.0
        for r, angle in valid_data:
            if r < min_dist:
                min_dist = r
                min_angle = angle

        # 충돌 임박 여부 판단
        collision_imminent = min_dist < self.collision_distance

        return collision_imminent, min_dist, min_angle

    def handle_imminent_collision(self, min_dist, obstacle_direction):
        """충돌 임박 시 회피 동작 (SLAM drift 최소화)"""
        self.get_logger().warn(f'COLLISION WARNING! Distance: {min_dist:.2f}m, Direction: {math.degrees(obstacle_direction):.0f}°')

        # 1. 현재 목표를 실패로 마킹 (같은 목표로 다시 안 가도록!)
        if self.current_goal:
            gx, gy = self.world_to_grid(self.current_goal[0], self.current_goal[1])
            if gx is not None:
                # 더 넓은 영역 차단 (// 3 대신 // 2)
                self.failed_goals.add((gx // 2, gy // 2))
                self.unreachable_frontiers.add((gx // 3, gy // 3))
                self.get_logger().info(f'Marked goal ({self.current_goal[0]:.1f}, {self.current_goal[1]:.1f}) as blocked')

        # 2. 현재 네비게이션 취소
        if self.goal_handle and self.is_navigating:
            self.goal_handle.cancel_goal_async()

        # 2. 즉시 정지
        twist = Twist()
        self.cmd_pub.publish(twist)
        time.sleep(0.2)  # SLAM 안정화 대기

        # 3. 천천히 후진 (SLAM drift 방지)
        twist.linear.x = -0.08  # 속도 낮춤
        for _ in range(6):  # 0.6초간 후진
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

        # 4. 정지 및 SLAM 안정화
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(0.3)  # SLAM이 따라갈 시간

        # 5. 장애물 반대 방향으로 천천히 회전
        turn_direction = -1.0 if obstacle_direction > 0 else 1.0

        # 더 안전한 방향 선택 (좌우 공간 비교)
        left_clearance, right_clearance = self.get_side_clearance()
        if left_clearance > right_clearance + 0.3:
            turn_direction = 1.0  # 왼쪽으로
        elif right_clearance > left_clearance + 0.3:
            turn_direction = -1.0  # 오른쪽으로

        twist.angular.z = turn_direction * 0.3  # 회전 속도 낮춤
        for _ in range(8):  # 0.8초간 회전
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

        # 6. 정지 및 SLAM 안정화
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(0.3)  # SLAM 안정화

        # 7. 네비게이션 상태 리셋 + 실패 카운트 증가
        self.is_navigating = False
        self.consecutive_failures += 1

        # 8. 다음 탐색 때 더 먼 거리 시도하도록
        self.current_distance_stage = min(
            self.current_distance_stage + 1,
            len(self.distance_stages) - 1
        )

    def get_side_clearance(self):
        """좌우 공간 여유 확인"""
        if self.last_scan is None:
            return 1.0, 1.0

        ranges = list(self.last_scan.ranges)
        num_ranges = len(ranges)

        # 왼쪽 (90도 부근)
        left_idx = num_ranges // 4
        left_range = 20
        left_data = ranges[max(0, left_idx - left_range):left_idx + left_range]
        left_valid = [r for r in left_data if 0.1 < r < 12.0]
        left_clearance = sum(left_valid) / len(left_valid) if left_valid else 0.0

        # 오른쪽 (-90도 부근)
        right_idx = 3 * num_ranges // 4
        right_data = ranges[max(0, right_idx - left_range):min(num_ranges, right_idx + left_range)]
        right_valid = [r for r in right_data if 0.1 < r < 12.0]
        right_clearance = sum(right_valid) / len(right_valid) if right_valid else 0.0

        return left_clearance, right_clearance

    # ==================== 개선 8: 개방도 계산 (벽 회피) ====================

    def calculate_openness(self, gx, gy):
        """frontier 주변의 개방도 계산 (벽 근처면 낮은 점수, 창고 환경 고려)"""
        if self.current_map is None:
            return 0.5

        h, w = self.current_map.shape
        radius = 12  # 검사 반경 축소 (15 → 12, ~0.6m) - 좁은 통로 허용

        y_min = max(0, gy - radius)
        y_max = min(h, gy + radius + 1)
        x_min = max(0, gx - radius)
        x_max = min(w, gx + radius + 1)

        region = self.current_map[y_min:y_max, x_min:x_max]
        total_cells = region.size

        if total_cells == 0:
            return 0.5

        # 각 셀 타입 개수
        free_cells = np.sum(region == 0)
        obstacle_cells = np.sum(region == 100)
        unknown_cells = np.sum(region == -1)

        # 개방도 = (빈 공간 + 미탐색) / 전체 - 장애물 페널티 (완화)
        openness = (free_cells + unknown_cells * 0.7) / total_cells  # 미탐색 가중치 증가
        obstacle_penalty = (obstacle_cells / total_cells) * 1.0  # 페널티 완화 (2.0 → 1.0)

        # 최소 0.3 보장 (완전 차단 방지)
        score = max(0.3, min(1.0, openness - obstacle_penalty))
        return score

    def is_frontier_near_wall(self, gx, gy, threshold=8):
        """frontier가 벽 근처인지 확인"""
        if self.current_map is None:
            return False

        h, w = self.current_map.shape

        # 주변 영역 검사
        y_min = max(0, gy - threshold)
        y_max = min(h, gy + threshold + 1)
        x_min = max(0, gx - threshold)
        x_max = min(w, gx + threshold + 1)

        region = self.current_map[y_min:y_max, x_min:x_max]
        obstacle_count = np.sum(region == 100)

        # 장애물이 많으면 벽 근처
        return obstacle_count > (threshold * threshold * 0.3)

    # ==================== 개선 6: Oscillation 감지 ====================

    def detect_oscillation(self):
        """왔다갔다 패턴 감지"""
        # 최소 1개 이상의 goal 시도 후에만 감지 (시작 시 오탐 방지)
        if self.total_goals < 1:
            return False

        if len(self.position_history) < 10:
            return False

        # 최근 위치들을 분석
        recent_positions = list(self.position_history)[-15:]

        # 위치별 방문 횟수 계산
        position_counts = {}
        for pos in recent_positions:
            if pos in position_counts:
                position_counts[pos] += 1
            else:
                position_counts[pos] = 1

        # 같은 위치를 여러 번 방문했는지 확인
        for pos, count in position_counts.items():
            if count >= self.oscillation_threshold:
                self.get_logger().warn(f'Oscillation detected at {pos}! Visited {count} times')
                return True

        # A-B-A-B 패턴 감지 (두 위치 사이를 왔다갔다)
        if len(recent_positions) >= 6:
            last_6 = recent_positions[-6:]
            # 패턴: A B A B A B 또는 유사
            unique_positions = set(last_6)
            if len(unique_positions) <= 2:
                self.get_logger().warn(f'A-B oscillation pattern detected!')
                return True

        return False

    def handle_oscillation(self):
        """Oscillation 발생 시 처리"""
        self.get_logger().warn('Handling oscillation - aggressive recovery')

        # 1. 현재 목표 취소
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()

        # 2. 더 강한 후진
        twist = Twist()
        twist.linear.x = -0.15
        for _ in range(15):  # 1.5초간 후진
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)

        # 3. 랜덤 방향 회전
        import random
        rotation_dir = random.choice([-1, 1])
        twist.angular.z = rotation_dir * 0.8  # 더 빠른 회전
        rotation_time = random.uniform(1.0, 2.5)  # 랜덤 회전 시간
        start = time.time()
        while time.time() - start < rotation_time:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        # 4. Costmap 클리어
        self.clear_costmaps()

        # 5. 캐시 초기화
        self.failed_goals.clear()
        self.unreachable_frontiers.clear()
        self.position_history.clear()
        self.consecutive_failures = 0
        self.current_distance_stage = 0
        self.oscillation_detected = False

        time.sleep(0.5)

    # ==================== 개선 5: Information Gain ====================

    def calculate_information_gain(self, gx, gy):
        """해당 위치에서 볼 수 있는 미탐색 영역 계산"""
        if self.current_map is None:
            return 0

        h, w = self.current_map.shape
        radius = self.info_gain_radius

        # 원형 마스크 생성
        y_min = max(0, gy - radius)
        y_max = min(h, gy + radius + 1)
        x_min = max(0, gx - radius)
        x_max = min(w, gx + radius + 1)

        # 해당 영역 추출
        region = self.current_map[y_min:y_max, x_min:x_max]

        # 미탐색 셀 개수
        unknown_count = np.sum(region == -1)

        # 거리 가중치 (가까운 셀일수록 더 가치 있음)
        cy, cx = gy - y_min, gx - x_min
        y_coords, x_coords = np.ogrid[0:region.shape[0], 0:region.shape[1]]
        distances = np.sqrt((y_coords - cy)**2 + (x_coords - cx)**2)
        distances = np.maximum(distances, 1)  # 0으로 나누기 방지

        # 가중치 적용된 information gain
        unknown_mask = (region == -1)
        weighted_gain = np.sum(unknown_mask / distances)

        return weighted_gain

    def calculate_information_gain_raycast(self, gx, gy):
        """레이캐스팅 기반 information gain (더 정확하지만 느림)"""
        if self.current_map is None:
            return 0

        h, w = self.current_map.shape
        total_gain = 0
        num_rays = 36  # 10도 간격
        max_range = self.info_gain_radius

        for i in range(num_rays):
            angle = 2 * math.pi * i / num_rays
            dx = math.cos(angle)
            dy = math.sin(angle)

            for r in range(1, max_range + 1):
                rx = int(gx + dx * r)
                ry = int(gy + dy * r)

                if not (0 <= rx < w and 0 <= ry < h):
                    break

                cell = self.current_map[ry, rx]

                if cell == 100:  # 장애물 - 레이 종료
                    break
                elif cell == -1:  # 미탐색 - gain 추가
                    total_gain += 1.0 / r  # 거리 가중치

        return total_gain

    # ==================== 경로 검증 ====================

    def check_path_exists(self, goal_x, goal_y, timeout=2.0):
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
        goal_msg.start.pose.position.x = self.robot_x
        goal_msg.start.pose.position.y = self.robot_y
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

        start_time = time.time()
        while not result_future.done():
            if time.time() - start_time > timeout:
                return True, None
            time.sleep(0.05)

        result = result_future.result()

        if result.result.path.poses:
            path_length = self.calculate_path_length(result.result.path)
            return True, path_length
        else:
            return False, None

    def calculate_path_length(self, path):
        if not path.poses or len(path.poses) < 2:
            return 0.0

        length = 0.0
        for i in range(1, len(path.poses)):
            dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x
            dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y
            length += math.sqrt(dx*dx + dy*dy)
        return length

    # ==================== 웨이포인트 ====================

    def get_waypoints_to_goal(self, goal_x, goal_y):
        dist = math.sqrt((goal_x - self.robot_x)**2 + (goal_y - self.robot_y)**2)

        if dist <= self.waypoint_interval * 1.5:
            return [(goal_x, goal_y)]

        num_waypoints = int(dist / self.waypoint_interval)

        waypoints = []
        for i in range(1, num_waypoints + 1):
            ratio = i / (num_waypoints + 1)
            wx = self.robot_x + ratio * (goal_x - self.robot_x)
            wy = self.robot_y + ratio * (goal_y - self.robot_y)
            waypoints.append((wx, wy))

        waypoints.append((goal_x, goal_y))
        return waypoints

    def send_waypoints(self, waypoints):
        if len(waypoints) == 1:
            self.send_goal(waypoints[0][0], waypoints[0][1])
            return

        if not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.send_goal(waypoints[0][0], waypoints[0][1])
            return

        self.is_navigating = True
        self.current_goal = waypoints[-1]
        self.goal_start_time = time.time()
        self.nav_start_pos = (self.robot_x, self.robot_y)
        self.total_goals += 1

        goal_msg = NavigateThroughPoses.Goal()

        for wx, wy in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wx
            pose.pose.position.y = wy

            angle = math.atan2(wy - self.robot_y, wx - self.robot_x)
            pose.pose.orientation.z = math.sin(angle / 2)
            pose.pose.orientation.w = math.cos(angle / 2)

            goal_msg.poses.append(pose)

        self.get_logger().info(f'Sending {len(waypoints)} waypoints')

        future = self.nav_through_poses_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    # ==================== Frontier 탐지 ====================

    def find_frontiers_fast(self):
        if self.current_map is None:
            return []

        free = (self.current_map == 0)
        unknown = (self.current_map == -1)
        unknown_dilated = ndimage.binary_dilation(unknown, iterations=1)
        frontier_mask = free & unknown_dilated

        occupied = (self.current_map == 100)
        obstacle_nearby = ndimage.binary_dilation(occupied, iterations=self.safety_margin)
        frontier_mask = frontier_mask & ~obstacle_nearby

        frontier_points = np.argwhere(frontier_mask)
        return frontier_points

    def cluster_frontiers(self, frontier_points):
        if len(frontier_points) == 0:
            return []

        frontier_mask = np.zeros_like(self.current_map, dtype=bool)
        for y, x in frontier_points:
            frontier_mask[y, x] = True

        labeled, num_features = ndimage.label(frontier_mask)

        clusters = []
        for i in range(1, num_features + 1):
            points = np.argwhere(labeled == i)
            if len(points) >= self.min_frontier_size:
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
        if self.current_map is None or self.map_info is None:
            return False

        h, w = self.current_map.shape
        if not (0 <= gx < w and 0 <= gy < h):
            return False

        if self.current_map[gy, gx] != 0:
            return False

        if (gx // 5, gy // 5) in self.failed_goals:
            return False

        margin = self.safety_margin
        y_min, y_max = max(0, gy - margin), min(h, gy + margin + 1)
        x_min, x_max = max(0, gx - margin), min(w, gx + margin + 1)
        region = self.current_map[y_min:y_max, x_min:x_max]
        if np.any(region == 100):
            return False

        return True

    def find_nearest_valid_point(self, gx, gy, max_search=20):
        for r in range(max_search):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) == r or abs(dy) == r:
                        nx, ny = gx + dx, gy + dy
                        if self.is_goal_valid(nx, ny):
                            return nx, ny
        return None, None

    # ==================== Frontier 선택 (개선 2 + 5) ====================

    def select_best_frontier_with_dynamic_distance(self, clusters):
        if not clusters:
            return None

        robot_gx, robot_gy = self.world_to_grid(self.robot_x, self.robot_y)
        if robot_gx is None:
            return None

        for stage_idx in range(self.current_distance_stage, len(self.distance_stages)):
            max_dist = self.distance_stages[stage_idx]
            min_dist = self.min_goal_distance

            best = self.find_frontier_in_range(clusters, min_dist, max_dist)

            if best:
                self.get_logger().info(f'Found frontier at stage {stage_idx} (max {max_dist}m)')
                self.current_distance_stage = max(0, stage_idx - 1)
                return best

        self.current_distance_stage = 0
        return None

    def find_frontier_in_range(self, clusters, min_dist, max_dist):
        best = None
        best_score = -1

        for cluster in clusters:
            cx, cy = cluster['center_grid']

            vx, vy = self.find_nearest_valid_point(cx, cy)
            if vx is None:
                continue

            wx, wy = self.grid_to_world(vx, vy)
            if wx is None:
                continue

            euclidean_dist = math.sqrt((wx - self.robot_x)**2 + (wy - self.robot_y)**2)

            if euclidean_dist < min_dist or euclidean_dist > max_dist:
                continue

            frontier_key = (vx // 3, vy // 3)
            if frontier_key in self.unreachable_frontiers:
                continue

            path_exists, path_length = self.check_path_exists(wx, wy, timeout=0.8)  # 1.5 → 0.8초

            if not path_exists:
                self.unreachable_frontiers.add(frontier_key)
                continue

            actual_dist = path_length if path_length else euclidean_dist

            # 개선 5: Information Gain 계산
            info_gain = self.calculate_information_gain(vx, vy)
            info_gain_normalized = min(info_gain / 100.0, 1.0)  # 정규화

            # 개선 8: 개방도 계산 (벽 근처 약한 페널티)
            openness = self.calculate_openness(vx, vy)

            # 벽 근처 frontier는 약한 페널티 (창고 등 좁은 환경 대응)
            if self.is_frontier_near_wall(vx, vy):
                openness *= 0.6  # 40% 페널티 (70% → 40%로 완화)

            # 점수 계산
            dist_score = 1.0 / (actual_dist + 0.1)
            size_score = min(cluster['size'] / 20.0, 1.0)

            # 단순화된 점수 계산 (4개 요소만)
            score = (info_gain_normalized * 0.35 +  # 정보량: 35% (미탐색 영역)
                     openness * 0.30 +              # 개방도: 30% (벽 회피)
                     dist_score * 0.20 +            # 거리: 20% (가까울수록)
                     size_score * 0.15)             # 크기: 15% (큰 frontier)

            if score > best_score:
                best_score = score
                best = {
                    'world': (wx, wy),
                    'grid': (vx, vy),
                    'euclidean_dist': euclidean_dist,
                    'path_dist': actual_dist,
                    'size': cluster['size'],
                    'info_gain': info_gain,
                    'openness': openness,
                    'score': score
                }

        return best

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # ==================== Recovery ====================

    def do_backup_move(self):
        self.get_logger().info('Recovery: Backing up...')
        twist = Twist()
        twist.linear.x = -0.1
        for _ in range(10):
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)

    def clear_costmaps(self):
        self.get_logger().info('Clearing costmaps...')

        if self.clear_global_costmap.wait_for_service(timeout_sec=1.0):
            req = ClearEntireCostmap.Request()
            self.clear_global_costmap.call_async(req)

        if self.clear_local_costmap.wait_for_service(timeout_sec=1.0):
            req = ClearEntireCostmap.Request()
            self.clear_local_costmap.call_async(req)

        time.sleep(0.5)

    def do_recovery_rotation(self):
        self.get_logger().info('Recovery: Rotating and clearing...')

        self.clear_costmaps()

        twist = Twist()
        twist.angular.z = 0.5
        for _ in range(15):
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        time.sleep(0.3)
        self.clear_costmaps()

        self.unreachable_frontiers.clear()

    def get_random_free_goal(self):
        if self.current_map is None:
            return None

        free_cells = np.argwhere(self.current_map == 0)
        if len(free_cells) == 0:
            return None

        robot_gx, robot_gy = self.world_to_grid(self.robot_x, self.robot_y)
        if robot_gx is None:
            return None

        distances = np.sqrt((free_cells[:, 1] - robot_gx)**2 +
                           (free_cells[:, 0] - robot_gy)**2)

        valid_mask = (distances > 20) & (distances < 60)
        valid_cells = free_cells[valid_mask]

        if len(valid_cells) == 0:
            return None

        idx = np.random.randint(len(valid_cells))
        gy, gx = valid_cells[idx]

        if self.is_goal_valid(gx, gy):
            return self.grid_to_world(gx, gy)

        return None

    # ==================== Main Loop ====================

    def explore_callback(self):
        if self.is_navigating:
            return

        if self.current_map is None:
            self.get_logger().info('Waiting for map...')
            return

        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            return

        # 진행 상황 로깅
        elapsed = time.time() - (self.exploration_start_time or time.time())
        success_rate = (self.successful_goals / max(1, self.total_goals)) * 100
        self.get_logger().info(
            f'Coverage: {self.last_coverage:.1f}% | '
            f'Goals: {self.successful_goals}/{self.total_goals} ({success_rate:.0f}%) | '
            f'Time: {elapsed:.0f}s'
        )

        if self.last_coverage > 95:
            self.get_logger().info('Exploration complete!')
            return

        # 개선 6: Oscillation 감지
        if self.detect_oscillation():
            self.handle_oscillation()
            return

        # Stuck 체크
        current_pos = (round(self.robot_x, 1), round(self.robot_y, 1))
        if current_pos == self.last_robot_pos:
            self.stuck_count += 1
        else:
            self.stuck_count = 0
        self.last_robot_pos = current_pos

        # Recovery
        if self.consecutive_failures >= 3 or self.stuck_count >= 5:
            self.get_logger().warn('Stuck detected! Recovery mode...')
            self.do_backup_move()
            self.do_recovery_rotation()
            self.consecutive_failures = 0
            self.stuck_count = 0
            self.failed_goals.clear()
            self.current_distance_stage = 0
            return

        if self.consecutive_failures >= 1:
            self.clear_costmaps()
            self.current_distance_stage = min(
                self.current_distance_stage + 1,
                len(self.distance_stages) - 1
            )

        # Frontier 탐지
        frontier_points = self.find_frontiers_fast()
        clusters = self.cluster_frontiers(frontier_points)

        goal = None
        path_dist = None
        info_gain = 0

        if clusters:
            best = self.select_best_frontier_with_dynamic_distance(clusters)
            if best:
                goal = best['world']
                path_dist = best.get('path_dist', best['euclidean_dist'])
                info_gain = best.get('info_gain', 0)
                self.get_logger().info(
                    f'Target: ({goal[0]:.2f}, {goal[1]:.2f}) '
                    f'dist={best["euclidean_dist"]:.1f}m path={path_dist:.1f}m '
                    f'size={best["size"]} gain={info_gain:.0f}'
                )

        if goal is None:
            self.get_logger().info('No frontier found, trying random goal...')
            result = self.get_random_free_goal()
            if result:
                goal = result
                path_dist = math.sqrt((goal[0] - self.robot_x)**2 + (goal[1] - self.robot_y)**2)

        if goal:
            if path_dist and path_dist > self.waypoint_interval * 2:
                waypoints = self.get_waypoints_to_goal(goal[0], goal[1])
                if len(waypoints) > 1:
                    self.get_logger().info(f'Using {len(waypoints)} waypoints')
                self.send_waypoints(waypoints)
            else:
                self.send_goal(goal[0], goal[1])
        else:
            self.get_logger().warn('No valid goal found')
            self.do_recovery_rotation()

    def send_goal(self, x, y):
        # 같은 목표 반복 감지
        target_key = (round(x, 1), round(y, 1))
        if target_key == self.last_target_key:
            self.same_target_count += 1
            if self.same_target_count >= 2:
                # 2번 이상 같은 목표 → 차단하고 다른 목표 찾기
                self.get_logger().warn(f'Same target {target_key} repeated {self.same_target_count} times, blocking!')
                gx, gy = self.world_to_grid(x, y)
                if gx is not None:
                    self.failed_goals.add((gx // 2, gy // 2))
                    self.unreachable_frontiers.add((gx // 3, gy // 3))
                self.same_target_count = 0
                self.last_target_key = None
                return  # 이 목표는 보내지 않음
        else:
            self.last_target_key = target_key
            self.same_target_count = 1

        self.is_navigating = True
        self.current_goal = (x, y)
        self.goal_start_time = time.time()
        self.nav_start_pos = (self.robot_x, self.robot_y)
        self.total_goals += 1

        # 목표 방향 저장 (다음 목표 선택 시 연속성 보너스용)
        self.last_goal_direction = math.atan2(y - self.robot_y, x - self.robot_x)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

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
            self.get_logger().info('Goal reached!')
            self.consecutive_failures = 0
            self.successful_goals += 1
            self.current_distance_stage = max(0, self.current_distance_stage - 1)
        else:
            self.get_logger().warn(f'Goal failed (status: {status})')
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
        success_rate = (node.successful_goals / max(1, node.total_goals)) * 100
        node.get_logger().info(
            f'\n=== Final Stats ===\n'
            f'Coverage: {node.last_coverage:.1f}%\n'
            f'Goals: {node.successful_goals}/{node.total_goals} ({success_rate:.0f}%)'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
