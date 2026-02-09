# Drobot

**Drobot** = **D**rone + **Robot** 하이브리드 로봇 시뮬레이션

ROS 2 Jazzy + Gazebo Harmonic 기반, 4바퀴 스키드 스티어 주행 + 쿼드콥터 비행이 가능한 하이브리드 로봇 프로젝트

## 패키지 구조

```
version a/
├── drobot_description/   # 로봇 모델 (URDF, meshes, worlds)
├── drobot_navigation/    # 네비게이션 노드 (goal_navigator, rule engine)
└── drobot_bringup/       # 런치 파일 + 설정 (모드별 config)
```

| 패키지 | 역할 |
|--------|------|
| `drobot_description` | URDF, STL 메시, Gazebo 월드, Jinja2 월드 생성기 |
| `drobot_navigation` | 목표 기반 자율주행 노드 + 규칙 엔진 |
| `drobot_bringup` | 런치 파일, Nav2/SLAM/EKF 설정, RViz 설정 |

## 빠른 시작

### 1. 설치

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-common \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization

cd ~/drobot/"version a"
colcon build --symlink-install
source install/setup.bash
```

### 2. 실행

```bash
# 터미널 열 때마다
source install/setup.bash

# 기본 실행 (empty 월드)
ros2 launch drobot_bringup navigation.launch.py

# 월드 변경
ros2 launch drobot_bringup navigation.launch.py world:=warehouse
ros2 launch drobot_bringup navigation.launch.py world:=f1_circuit
ros2 launch drobot_bringup navigation.launch.py world:=office_maze
```

## 사용 가능한 월드

### 기본 월드

| 월드 | 설명 | 명령어 |
|------|------|--------|
| `empty` | 장애물 있는 방 (기본) | `world:=empty` |
| `full_world` | 복합 환경 | `world:=full_world` |
| `simple_room` | 거실 | `world:=simple_room` |
| `warehouse` | 창고 | `world:=warehouse` |
| `office_maze` | 사무실 미로 | `world:=office_maze` |
| `f1_circuit` | F1 서킷 | `world:=f1_circuit` |

### 생성된 월드 (랜덤 장애물)

| 월드 | 설명 | 명령어 |
|------|------|--------|
| `room_generated` | 15x15 랜덤 장애물 방 | `world:=room_generated` |
| `maze_generated` | 20x20 랜덤 미로 | `world:=maze_generated` |
| `road_test` | 도로 + 움직이는 차량 | `world:=road_test` |

## 목표 설정

RViz에서 "2D Goal Pose" 버튼 클릭 후 맵에서 위치 지정

또는 터미널에서:
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}
}"
```

## 월드 생성기 (Jinja2 + YAML)

SDF 템플릿 + YAML config 조합으로 월드를 생성합니다.

### Interactive 모드 (대화형)

```bash
cd drobot_description/scripts
python3 generate_world.py -i
```

단계별 프롬프트로 월드 이름 → 벽 → 스폰 → 장애물 → 도로 → 저장 순서로 진행됩니다.
YAML config + SDF 파일이 쌍으로 저장됩니다.

### YAML config 모드

```bash
cd drobot_description/scripts

# YAML config로 생성
python3 generate_world.py ../worlds/configs/example.yaml -o my_world

# 생성 후 빌드 & 실행
cd ~/drobot/"version a" && colcon build --symlink-install
source install/setup.bash
ros2 launch drobot_bringup navigation.launch.py world:=my_world
```

### YAML config 예시

```yaml
world:
  name: my_room
  ground_color: gray
  walls:
    enabled: true
    size: {w: 10, h: 10}
    height: 1.0
    doors: [south]
  spawn:
    pos: {x: 0, y: 0}
    safe_radius: 1.5
  obstacles:
    static:
      - model: box
        pos: {x: 3, y: 3}
        size: {x: 0.5, y: 0.5, z: 0.8}
        color: red
      - model: cylinder
        pos: {x: -2, y: 1}
        radius: 0.3
        height: 0.6
        color: green
    moving:
      - model: box
        pos: {x: 2, y: -2}
        size: {x: 0.4, y: 0.4, z: 0.6}
        color: yellow
    random:
      count: 5
      models: [box, cylinder]
      size_range: [0.3, 0.8]
      seed: 42
```

## 프로젝트 구조

```
drobot_description/
├── urdf/drobot.urdf.xacro       # 로봇 모델 (URDF)
├── meshes/drobot_body.stl        # 3D 메시 (20MB)
├── models/                       # Jinja2 SDF 템플릿
│   ├── base/world.sdf.j2        #   월드 뼈대
│   ├── obstacles/*.sdf.j2        #   박스, 실린더, 벽
│   └── environment/*.sdf.j2      #   도로, 차선 마킹
├── worlds/                       # Gazebo 월드 파일
│   ├── *.sdf                     #   수동 생성 월드
│   ├── generated/*.sdf           #   자동 생성 월드
│   └── configs/*.yaml            #   YAML 월드 정의
└── scripts/generate_world.py     # 월드 생성기

drobot_navigation/
└── drobot_navigation/
    ├── goal_navigator.py         # 목표 기반 자율주행 노드
    ├── config.py                 # 상수 설정
    └── rules/engine.py           # YAML 규칙 엔진

drobot_bringup/
├── launch/navigation.launch.py  # 올인원 런치
└── config/
    ├── common/                   # EKF, SLAM, Gazebo 브릿지
    ├── navigation/               # Nav2, BT, rules, RViz
    └── spawn_positions.yaml      # 월드별 스폰 위치
```

## 명령어 요약

| 상황 | 명령어 |
|------|--------|
| 터미널 열었을 때 | `source install/setup.bash` |
| 코드 수정 후 | `colcon build && source install/setup.bash` |
| 실행 | `ros2 launch drobot_bringup navigation.launch.py` |
| 월드 변경 | `... world:=warehouse` |
| 키보드 조종 | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |
| 프로세스 종료 | `pkill -9 -f "gz\|rviz"` |
