#!/usr/bin/env python3
"""
World Generator for Drobot

프로시저럴 월드 생성기 - 벽, 장애물, 움직이는 물체, 도로 등을
파이썬 코드로 생성하여 SDF 파일로 출력합니다.

Usage:
    from world_generator import WorldGenerator

    gen = WorldGenerator(name="my_world", room_size=(10, 10))
    gen.add_walls()
    gen.add_random_obstacles(count=5)
    gen.add_moving_obstacle(path=[(0, 2), (0, -2)], speed=0.5)
    gen.save("my_world.sdf")
"""
import random
import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import os


@dataclass
class Color:
    """RGBA 색상"""
    r: float
    g: float
    b: float
    a: float = 1.0

    def to_sdf(self) -> str:
        return f"{self.r} {self.g} {self.b} {self.a}"


@dataclass
class Pose:
    """6DOF 포즈 (x, y, z, roll, pitch, yaw)"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def to_sdf(self) -> str:
        return f"{self.x} {self.y} {self.z} {self.roll} {self.pitch} {self.yaw}"


@dataclass
class Material:
    """재질 정의"""
    ambient: Color
    diffuse: Optional[Color] = None
    specular: Optional[Color] = None

    def __post_init__(self):
        if self.diffuse is None:
            self.diffuse = self.ambient

    def to_sdf(self) -> str:
        sdf = f"""          <material>
            <ambient>{self.ambient.to_sdf()}</ambient>
            <diffuse>{self.diffuse.to_sdf()}</diffuse>"""
        if self.specular:
            sdf += f"\n            <specular>{self.specular.to_sdf()}</specular>"
        sdf += "\n          </material>"
        return sdf


# 미리 정의된 색상들
class Colors:
    GRAY = Color(0.5, 0.5, 0.5)
    WHITE = Color(0.9, 0.9, 0.9)
    RED = Color(0.8, 0.2, 0.2)
    GREEN = Color(0.2, 0.6, 0.2)
    BLUE = Color(0.2, 0.2, 0.8)
    YELLOW = Color(0.8, 0.8, 0.2)
    ORANGE = Color(0.8, 0.4, 0.1)
    BROWN = Color(0.4, 0.25, 0.1)
    DARK_GRAY = Color(0.3, 0.3, 0.3)
    ASPHALT = Color(0.2, 0.2, 0.2)
    ROAD_YELLOW = Color(0.9, 0.8, 0.1)
    ROAD_WHITE = Color(0.95, 0.95, 0.95)


class WorldGenerator:
    """프로시저럴 월드 생성기"""

    def __init__(
        self,
        name: str = "generated_world",
        room_size: Tuple[float, float] = (10.0, 10.0),
        spawn_point: Tuple[float, float] = (0.0, 0.0),
        spawn_safe_radius: float = 1.5,
        min_obstacle_gap: float = 4.0,
        wall_height: float = 1.0,
        wall_thickness: float = 0.1,
        seed: Optional[int] = None
    ):
        """
        Args:
            name: 월드 이름
            room_size: (width, height) 방 크기 (미터)
            spawn_point: (x, y) 로봇 스폰 위치
            spawn_safe_radius: 스폰 주변 안전 반경 (장애물 배치 금지)
            min_obstacle_gap: 장애물 간 최소 간격 (로봇 통과용, 기본 4m)
            wall_height: 벽 높이
            wall_thickness: 벽 두께
            seed: 랜덤 시드 (재현성)
        """
        self.name = name
        self.room_width, self.room_height = room_size
        self.spawn_x, self.spawn_y = spawn_point
        self.spawn_safe_radius = spawn_safe_radius
        self.min_obstacle_gap = min_obstacle_gap
        self.wall_height = wall_height
        self.wall_thickness = wall_thickness

        if seed is not None:
            random.seed(seed)

        self.models: List[str] = []
        self.model_count = 0
        self._occupied_positions: List[Tuple[float, float, float]] = []  # (x, y, radius)

    def _is_position_safe(self, x: float, y: float, radius: float) -> bool:
        """위치가 안전한지 확인 (스폰 포인트 및 다른 물체와 충돌 안 함)"""
        # 스폰 포인트 체크
        dist_to_spawn = math.sqrt((x - self.spawn_x)**2 + (y - self.spawn_y)**2)
        if dist_to_spawn < self.spawn_safe_radius + radius:
            return False

        # 다른 물체와 충돌 체크 (최소 간격 유지)
        for ox, oy, or_ in self._occupied_positions:
            dist = math.sqrt((x - ox)**2 + (y - oy)**2)
            if dist < or_ + radius + self.min_obstacle_gap:  # 로봇 통과용 간격
                return False

        return True

    def _find_safe_position(
        self,
        radius: float,
        margin: float = 0.5,
        max_attempts: int = 100
    ) -> Optional[Tuple[float, float]]:
        """안전한 랜덤 위치 찾기"""
        half_w = self.room_width / 2 - margin - radius
        half_h = self.room_height / 2 - margin - radius

        for _ in range(max_attempts):
            x = random.uniform(-half_w, half_w)
            y = random.uniform(-half_h, half_h)
            if self._is_position_safe(x, y, radius):
                return (x, y)

        return None

    def _generate_model_name(self, prefix: str) -> str:
        """고유한 모델 이름 생성"""
        self.model_count += 1
        return f"{prefix}_{self.model_count}"

    # ==================== 기본 요소 ====================

    def _sdf_header(self) -> str:
        """SDF 헤더 (physics, plugins, lighting)"""
        return f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="{self.name}">

    <!-- Physics -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
'''

    def _sdf_ground(self, color: Color = Colors.GRAY) -> str:
        """바닥면"""
        mat = Material(ambient=color)
        return f'''
    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
{mat.to_sdf()}
        </visual>
      </link>
    </model>
'''

    def _sdf_footer(self) -> str:
        """SDF 푸터"""
        return '''
  </world>
</sdf>
'''

    # ==================== 벽 ====================

    def add_walls(self, color: Color = Colors.GRAY, doors: List[str] = None):
        """
        방 외벽 추가

        Args:
            color: 벽 색상
            doors: 문 위치 리스트 ["north", "south", "east", "west"]
        """
        doors = doors or []
        half_w = self.room_width / 2
        half_h = self.room_height / 2
        h = self.wall_height
        t = self.wall_thickness

        walls = []

        # 북쪽 벽
        if "north" not in doors:
            walls.append(("wall_north", Pose(0, half_h, h/2), (self.room_width, t, h)))
        else:
            # 문 있으면 양쪽으로 나눔
            door_width = 1.0
            side_w = (self.room_width - door_width) / 2
            walls.append(("wall_north_left", Pose(-self.room_width/4 - door_width/4, half_h, h/2), (side_w, t, h)))
            walls.append(("wall_north_right", Pose(self.room_width/4 + door_width/4, half_h, h/2), (side_w, t, h)))

        # 남쪽 벽
        if "south" not in doors:
            walls.append(("wall_south", Pose(0, -half_h, h/2), (self.room_width, t, h)))
        else:
            door_width = 1.0
            side_w = (self.room_width - door_width) / 2
            walls.append(("wall_south_left", Pose(-self.room_width/4 - door_width/4, -half_h, h/2), (side_w, t, h)))
            walls.append(("wall_south_right", Pose(self.room_width/4 + door_width/4, -half_h, h/2), (side_w, t, h)))

        # 동쪽 벽
        if "east" not in doors:
            walls.append(("wall_east", Pose(half_w, 0, h/2), (t, self.room_height, h)))
        else:
            door_width = 1.0
            side_h = (self.room_height - door_width) / 2
            walls.append(("wall_east_top", Pose(half_w, self.room_height/4 + door_width/4, h/2), (t, side_h, h)))
            walls.append(("wall_east_bottom", Pose(half_w, -self.room_height/4 - door_width/4, h/2), (t, side_h, h)))

        # 서쪽 벽
        if "west" not in doors:
            walls.append(("wall_west", Pose(-half_w, 0, h/2), (t, self.room_height, h)))
        else:
            door_width = 1.0
            side_h = (self.room_height - door_width) / 2
            walls.append(("wall_west_top", Pose(-half_w, self.room_height/4 + door_width/4, h/2), (t, side_h, h)))
            walls.append(("wall_west_bottom", Pose(-half_w, -self.room_height/4 - door_width/4, h/2), (t, side_h, h)))

        for name, pose, size in walls:
            self.add_box(name=name, pose=pose, size=size, color=color, static=True, track_occupied=False)

    def add_inner_wall(
        self,
        start: Tuple[float, float],
        end: Tuple[float, float],
        color: Color = Colors.BROWN,
        name: Optional[str] = None
    ):
        """
        내부 벽 추가 (두 점 사이)

        Args:
            start: 시작점 (x, y)
            end: 끝점 (x, y)
            color: 벽 색상
        """
        x1, y1 = start
        x2, y2 = end

        length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        angle = math.atan2(y2 - y1, x2 - x1)
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        name = name or self._generate_model_name("inner_wall")
        pose = Pose(cx, cy, self.wall_height / 2, yaw=angle)
        size = (length, self.wall_thickness, self.wall_height)

        self.add_box(name=name, pose=pose, size=size, color=color, static=True)

    # ==================== 장애물 ====================

    def add_box(
        self,
        pose: Pose,
        size: Tuple[float, float, float],
        color: Color = Colors.RED,
        name: Optional[str] = None,
        static: bool = True,
        track_occupied: bool = True
    ):
        """박스 장애물 추가"""
        name = name or self._generate_model_name("box")
        mat = Material(ambient=color)

        sdf = f'''
    <model name="{name}">
      <static>{str(static).lower()}</static>
      <pose>{pose.to_sdf()}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>
{mat.to_sdf()}
        </visual>
      </link>
    </model>
'''
        self.models.append(sdf)

        # 점유 위치 기록 (충돌 방지용) - 벽 등은 제외 가능
        if track_occupied:
            radius = max(size[0], size[1]) / 2
            self._occupied_positions.append((pose.x, pose.y, radius))

    def add_cylinder(
        self,
        pose: Pose,
        radius: float,
        height: float,
        color: Color = Colors.GREEN,
        name: Optional[str] = None,
        static: bool = True
    ):
        """실린더 장애물 추가"""
        name = name or self._generate_model_name("cylinder")
        mat = Material(ambient=color)

        sdf = f'''
    <model name="{name}">
      <static>{str(static).lower()}</static>
      <pose>{pose.to_sdf()}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{radius}</radius>
              <length>{height}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{radius}</radius>
              <length>{height}</length>
            </cylinder>
          </geometry>
{mat.to_sdf()}
        </visual>
      </link>
    </model>
'''
        self.models.append(sdf)
        self._occupied_positions.append((pose.x, pose.y, radius))

    def add_random_obstacles(
        self,
        count: int = 5,
        min_size: float = 0.3,
        max_size: float = 0.8,
        obstacle_types: List[str] = None
    ):
        """
        랜덤 장애물 추가 (스폰 포인트 회피)

        Args:
            count: 장애물 개수
            min_size: 최소 크기
            max_size: 최대 크기
            obstacle_types: ["box", "cylinder"] 중 선택
        """
        obstacle_types = obstacle_types or ["box", "cylinder"]
        colors = [Colors.RED, Colors.BLUE, Colors.GREEN, Colors.ORANGE, Colors.YELLOW]

        placed = 0
        for _ in range(count * 3):  # 최대 시도 횟수
            if placed >= count:
                break

            size = random.uniform(min_size, max_size)
            pos = self._find_safe_position(radius=size / 2)

            if pos is None:
                continue

            x, y = pos
            color = random.choice(colors)
            obs_type = random.choice(obstacle_types)

            if obs_type == "box":
                w = random.uniform(min_size, max_size)
                d = random.uniform(min_size, max_size)
                h = random.uniform(0.4, 0.8)
                self.add_box(
                    pose=Pose(x, y, h / 2),
                    size=(w, d, h),
                    color=color
                )
            else:
                r = random.uniform(min_size / 2, max_size / 2)
                h = random.uniform(0.4, 0.8)
                self.add_cylinder(
                    pose=Pose(x, y, h / 2),
                    radius=r,
                    height=h,
                    color=color
                )

            placed += 1

        print(f"[WorldGenerator] {placed}/{count} obstacles placed")

    # ==================== 움직이는 장애물 ====================

    def add_moving_obstacle(
        self,
        path: List[Tuple[float, float]],
        speed: float = 0.5,
        size: Tuple[float, float, float] = (0.4, 0.4, 0.6),
        color: Color = Colors.YELLOW,
        name: Optional[str] = None,
        loop: bool = True
    ):
        """
        움직이는 장애물 추가 (경로를 따라 이동)

        Args:
            path: [(x1, y1), (x2, y2), ...] 경로 포인트
            speed: 이동 속도 (m/s)
            size: (width, depth, height)
            color: 색상
            loop: 경로 반복 여부
        """
        if len(path) < 2:
            print("[Warning] Moving obstacle needs at least 2 path points")
            return

        name = name or self._generate_model_name("moving_obstacle")
        mat = Material(ambient=color)

        # 시작 위치
        start_x, start_y = path[0]
        z_pos = size[2] / 2

        # waypoints 문자열 생성 (Gazebo Harmonic 형식 - time + pose)
        # 각 waypoint까지의 누적 시간 계산
        waypoints = []
        total_time = 0.0
        for i, (x, y) in enumerate(path):
            if i > 0:
                prev_x, prev_y = path[i-1]
                dist = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)
                total_time += dist / speed
            waypoints.append(f'''          <waypoint>
            <time>{total_time:.2f}</time>
            <pose>{x} {y} {z_pos} 0 0 0</pose>
          </waypoint>''')
        waypoints_str = "\n".join(waypoints)

        # force 계산 (속도에 비례)
        force = max(50, speed * 100)

        sdf = f'''
    <!-- Moving Obstacle: {name} -->
    <model name="{name}">
      <static>false</static>
      <pose>{start_x} {start_y} {size[2]/2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>
{mat.to_sdf()}
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>
          </inertia>
        </inertial>
      </link>

      <!-- Trajectory Plugin (Gazebo Harmonic) -->
      <plugin filename="gz-sim-trajectory-follower-system" name="gz::sim::systems::TrajectoryFollower">
        <link_name>link</link_name>
        <force>{force}</force>
        <torque>50</torque>
        <loop>{"true" if loop else "false"}</loop>
{waypoints_str}
      </plugin>
    </model>
'''
        self.models.append(sdf)

    # ==================== 도로 ====================

    def add_road(
        self,
        start: Tuple[float, float],
        end: Tuple[float, float],
        width: float = 4.0,
        lanes: int = 2,
        name: Optional[str] = None
    ):
        """
        도로 추가 (차선 포함)

        Args:
            start: 시작점
            end: 끝점
            width: 도로 너비
            lanes: 차선 수
        """
        name = name or self._generate_model_name("road")

        x1, y1 = start
        x2, y2 = end

        length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        angle = math.atan2(y2 - y1, x2 - x1)
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        # 도로 바닥
        road_mat = Material(ambient=Colors.ASPHALT)

        sdf = f'''
    <!-- Road: {name} -->
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} 0.001 0 0 {angle}</pose>
      <link name="link">
        <visual name="road_surface">
          <geometry><box><size>{length} {width} 0.01</size></box></geometry>
{road_mat.to_sdf()}
        </visual>
      </link>
    </model>
'''
        self.models.append(sdf)

        # 중앙선 (노란색 점선)
        if lanes >= 2:
            self._add_lane_marking(
                cx, cy, length, angle,
                offset=0, color=Colors.ROAD_YELLOW,
                dashed=True, name=f"{name}_center_line"
            )

        # 양쪽 가장자리 선 (흰색 실선)
        self._add_lane_marking(
            cx, cy, length, angle,
            offset=width/2 - 0.1, color=Colors.ROAD_WHITE,
            dashed=False, name=f"{name}_edge_left"
        )
        self._add_lane_marking(
            cx, cy, length, angle,
            offset=-(width/2 - 0.1), color=Colors.ROAD_WHITE,
            dashed=False, name=f"{name}_edge_right"
        )

    def _add_lane_marking(
        self,
        cx: float, cy: float,
        length: float, angle: float,
        offset: float,
        color: Color,
        dashed: bool = False,
        name: str = "lane_marking"
    ):
        """차선 마킹 추가"""
        mat = Material(ambient=color)

        # offset 적용 (도로 방향에 수직으로)
        perp_angle = angle + math.pi / 2
        mx = cx + offset * math.cos(perp_angle)
        my = cy + offset * math.sin(perp_angle)

        if dashed:
            # 점선 (2m 간격, 1m 길이)
            dash_length = 1.0
            gap_length = 2.0
            num_dashes = int(length / (dash_length + gap_length))

            for i in range(num_dashes):
                dist = (i - num_dashes/2) * (dash_length + gap_length) + dash_length/2
                dx = mx + dist * math.cos(angle)
                dy = my + dist * math.sin(angle)

                sdf = f'''
    <model name="{name}_{i}">
      <static>true</static>
      <pose>{dx} {dy} 0.002 0 0 {angle}</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{dash_length} 0.1 0.01</size></box></geometry>
{mat.to_sdf()}
        </visual>
      </link>
    </model>
'''
                self.models.append(sdf)
        else:
            # 실선
            sdf = f'''
    <model name="{name}">
      <static>true</static>
      <pose>{mx} {my} 0.002 0 0 {angle}</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{length} 0.1 0.01</size></box></geometry>
{mat.to_sdf()}
        </visual>
      </link>
    </model>
'''
            self.models.append(sdf)

    def add_moving_car(
        self,
        path: List[Tuple[float, float]],
        speed: float = 2.0,
        color: Color = Colors.BLUE,
        name: Optional[str] = None
    ):
        """
        움직이는 차량 추가 (도로용)

        Args:
            path: 경로 포인트
            speed: 속도 (m/s)
            color: 차량 색상
        """
        name = name or self._generate_model_name("car")

        # 차량 크기 (간단한 박스)
        car_size = (2.0, 1.0, 0.8)

        self.add_moving_obstacle(
            path=path,
            speed=speed,
            size=car_size,
            color=color,
            name=name,
            loop=True
        )

    # ==================== 생성 ====================

    def generate(self, ground_color: Color = Colors.GRAY) -> str:
        """SDF 문자열 생성"""
        sdf = self._sdf_header()
        sdf += self._sdf_ground(ground_color)
        sdf += "\n".join(self.models)
        sdf += self._sdf_footer()
        return sdf

    def save(self, filename: str, output_dir: Optional[str] = None):
        """SDF 파일 저장"""
        if output_dir is None:
            output_dir = os.path.dirname(os.path.abspath(__file__))
            output_dir = os.path.join(output_dir, "..", "worlds", "generated")

        os.makedirs(output_dir, exist_ok=True)

        filepath = os.path.join(output_dir, filename)
        if not filepath.endswith('.sdf'):
            filepath += '.sdf'

        with open(filepath, 'w') as f:
            f.write(self.generate())

        print(f"[WorldGenerator] Saved: {filepath}")
        return filepath


# ==================== 프리셋 함수들 ====================

def create_simple_room(
    size: Tuple[float, float] = (8, 8),
    obstacles: int = 5,
    seed: int = None
) -> WorldGenerator:
    """간단한 방 생성"""
    gen = WorldGenerator(
        name="simple_room_generated",
        room_size=size,
        seed=seed
    )
    gen.add_walls()
    gen.add_random_obstacles(count=obstacles)
    return gen


def create_maze(
    size: Tuple[float, float] = (12, 12),
    complexity: int = 5,
    seed: int = None
) -> WorldGenerator:
    """미로 생성"""
    gen = WorldGenerator(
        name="maze_generated",
        room_size=size,
        spawn_point=(-size[0]/2 + 1, -size[1]/2 + 1),
        seed=seed
    )
    gen.add_walls()

    # 내부 벽 추가
    random.seed(seed)
    for _ in range(complexity):
        x1 = random.uniform(-size[0]/2 + 1, size[0]/2 - 1)
        y1 = random.uniform(-size[1]/2 + 1, size[1]/2 - 1)

        # 수평 또는 수직 벽
        if random.random() > 0.5:
            length = random.uniform(1.5, 3.0)
            x2, y2 = x1 + length, y1
        else:
            length = random.uniform(1.5, 3.0)
            x2, y2 = x1, y1 + length

        # 범위 클리핑
        x2 = max(min(x2, size[0]/2 - 0.5), -size[0]/2 + 0.5)
        y2 = max(min(y2, size[1]/2 - 0.5), -size[1]/2 + 0.5)

        gen.add_inner_wall((x1, y1), (x2, y2))

    return gen


def create_road_scene(
    road_length: float = 20.0,
    num_cars: int = 3,
    seed: int = None
) -> WorldGenerator:
    """도로 씬 생성"""
    gen = WorldGenerator(
        name="road_scene_generated",
        room_size=(road_length + 4, 10),
        spawn_point=(0, -3),
        seed=seed
    )

    # 도로 추가
    gen.add_road(
        start=(-road_length/2, 0),
        end=(road_length/2, 0),
        width=6.0,
        lanes=2
    )

    # 움직이는 차량들
    random.seed(seed)
    for i in range(num_cars):
        lane_y = 1.5 if i % 2 == 0 else -1.5
        start_x = random.uniform(-road_length/2 + 2, road_length/2 - 2)

        if lane_y > 0:
            # 오른쪽 차선: 오른쪽으로 이동
            path = [(start_x, lane_y), (road_length/2 - 1, lane_y), (-road_length/2 + 1, lane_y)]
        else:
            # 왼쪽 차선: 왼쪽으로 이동
            path = [(start_x, lane_y), (-road_length/2 + 1, lane_y), (road_length/2 - 1, lane_y)]

        color = random.choice([Colors.RED, Colors.BLUE, Colors.GREEN, Colors.WHITE])
        gen.add_moving_car(path=path, speed=random.uniform(1.5, 3.0), color=color)

    return gen


# ==================== CLI ====================

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Generate Gazebo world files")
    parser.add_argument("--type", choices=["room", "maze", "road"], default="room")
    parser.add_argument("--size", type=float, nargs=2, default=[10, 10])
    parser.add_argument("--obstacles", type=int, default=5)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--output", type=str, default=None)

    args = parser.parse_args()

    if args.type == "room":
        gen = create_simple_room(tuple(args.size), args.obstacles, args.seed)
    elif args.type == "maze":
        gen = create_maze(tuple(args.size), args.obstacles, args.seed)
    elif args.type == "road":
        gen = create_road_scene(args.size[0], args.obstacles, args.seed)

    output_name = args.output or f"{args.type}_generated.sdf"
    gen.save(output_name)
