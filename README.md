# Drobot

**Drobot** = **D**rone + **Robot** 하이브리드 로봇 시뮬레이션

ROS 2 Jazzy + Gazebo Harmonic 기반, 4바퀴 스키드 스티어 주행 + 쿼드콥터 비행이 가능한 하이브리드 로봇 프로젝트

## 패키지 구조

```
ros2_ws/src/
├── drobot_description/   # 로봇 모델 (URDF, meshes, worlds)
├── drobot_scan_2d/       # 2D 네비게이션 (goal_navigator, rule engine)
├── drobot_scan_3d/       # 3D 스캔/비행 (예정)
├── drobot_bringup/       # 런치 파일 + 설정 + World UI
├── drobot_controller/    # 로봇 컨트롤러 (예정)
├── drobot_simulation/    # 시뮬레이션 (예정)
├── drobot_strategy/      # 전략/의사결정 (예정)
└── drobot_yolo/          # YOLO 객체 인식 (예정)
```

| 패키지 | 빌드 | 역할 |
|--------|------|------|
| `drobot_description` | ament_cmake | URDF, 11개 STL 메시, Gazebo 플러그인, 월드 파일 (hospital, warehouse 등) |
| `drobot_scan_2d` | ament_cmake | 목표 기반 자율주행 노드 + YAML 규칙 엔진 |
| `drobot_bringup` | ament_python | 런치 파일 (navigation, bringup), Nav2/SLAM/EKF 설정, World UI (Tkinter) |
| `drobot_controller` | ament_python | (예정) |
| `drobot_scan_3d` | ament_python | (예정) |
| `drobot_simulation` | ament_cmake | (예정) |
| `drobot_strategy` | ament_python | (예정) |
| `drobot_yolo` | ament_python | (예정) |

## 빠른 시작

### 1. 의존성 설치

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-common \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard
```

### 2. 빌드

```bash
cd ~/Documents/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. 실행

```bash
# 자율주행 (Navigation)
ros2 launch drobot_bringup navigation.launch.py
ros2 launch drobot_bringup navigation.launch.py world:=hospital_original

# World UI (월드 생성 + 런치)
ros2 run drobot_bringup world_ui

# RViz 시각화만
ros2 launch drobot_description display.launch.py

# Gazebo 시뮬레이션만
ros2 launch drobot_description gazebo.launch.py
```

## 목표 설정

RViz에서 "2D Goal Pose" 버튼 클릭 후 맵에서 위치 지정

또는 터미널에서:
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}
}"
```

## 프로젝트 구조 상세

```
drobot_description/
├── urdf/
│   ├── drobot.urdf.xacro       # 메인 URDF (SW2URDF 기반)
│   ├── gazebo.xacro             # Gazebo 플러그인 (DiffDrive, JointPosition)
│   └── ros2_control.xacro       # ros2_control 인터페이스
├── meshes/                      # 11개 STL 메시 (base, arms, wheels, frames)
├── launch/
│   ├── display.launch.py        # RViz 시각화
│   └── gazebo.launch.py         # Gazebo 시뮬레이션
├── config/display.rviz
├── object/obstacle_library/     # 장애물 YAML 라이브러리
└── worlds/original/             # 사전 정의 월드 (hospital, warehouse, cafe 등)

drobot_scan_2d/
└── drobot_scan_2d/
    ├── goal_navigator.py         # Nav2 액션 클라이언트, 자율주행
    ├── config.py                 # 안전거리, 타임아웃 상수
    └── rules/engine.py           # YAML 규칙 엔진 (금지구역, 속도제한, 정지규칙)

drobot_bringup/
├── launch/
│   ├── navigation.launch.py     # Gazebo + SLAM + Nav2 + GoalNavigator 올인원
│   └── bringup.launch.py        # World UI용 런치
├── config/
│   ├── common/                   # EKF, SLAM, Gazebo 브릿지
│   ├── navigation/               # Nav2, BT, rules, RViz
│   └── spawn_positions.yaml      # 월드별 스폰 위치
└── drobot_bringup/
    ├── world_ui.py               # Tkinter 월드 런처 GUI
    └── worldgen.py               # SDF 월드 자동 생성기
```

## 명령어 요약

| 상황 | 명령어 |
|------|--------|
| 터미널 열었을 때 | `source install/setup.bash` |
| 코드 수정 후 | `colcon build --symlink-install && source install/setup.bash` |
| 자율주행 실행 | `ros2 launch drobot_bringup navigation.launch.py` |
| 월드 변경 | `... world:=<월드이름>` |
| World UI | `ros2 run drobot_bringup world_ui` |
| 키보드 조종 | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |
| 프로세스 종료 | `pkill -9 -f "gz\|rviz"` |
