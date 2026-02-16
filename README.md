# Drobot

**Drobot** = **D**rone + **Robot** 하이브리드 로봇 시뮬레이션

ROS 2 Jazzy + Gazebo Harmonic 기반, 4바퀴 스키드 스티어 주행 + 쿼드콥터 비행이 가능한 하이브리드 로봇 프로젝트

## 패키지 구조

```
ros2_ws/src/
├── drobot_description/   # 로봇 모델 (URDF, meshes, worlds, 3D models)
├── drobot_scan_2d/       # 2D 네비게이션 (goal_navigator, rule engine)
├── drobot_scan_3d/       # 3D 스캔/비행 (예정)
├── drobot_bringup/       # 런치 파일 + 설정 + World UI
├── drobot_controller/    # 로봇 컨트롤러 (예정)
├── drobot_simulator/     # 시뮬레이션 (예정)
├── drobot_strategy/      # 전략/의사결정 (예정)
└── perception/           # YOLO 객체 인식 (카메라 기반 음식 감지)
```

| 패키지 | 빌드 | 역할 |
|--------|------|------|
| `drobot_description` | ament_cmake | URDF, 11개 STL 메시, Gazebo 플러그인, 월드 파일, 98개 3D 모델 |
| `drobot_scan_2d` | ament_cmake | 목표 기반 자율주행 노드 + YAML 규칙 엔진 |
| `drobot_bringup` | ament_python | 런치 파일 (navigation, bringup), Nav2/SLAM/EKF 설정, World UI |
| `perception` | ament_python | YOLO 기반 객체 인식 (카메라 → 음식 감지) |
| `drobot_controller` | ament_python | (예정) |
| `drobot_scan_3d` | ament_python | (예정) |
| `drobot_simulator` | ament_cmake | (예정) |
| `drobot_strategy` | ament_python | (예정) |

## 빠른 시작

### 1. 클론

```bash
cd ~/Documents
git clone -b kangjun-version2 https://github.com/protkjj/drobot.git ros2_ws/src
```

### 2. 의존성 설치

```bash
# ROS 2 패키지
sudo apt update && sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-common \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro \
  ros-jazzy-cv-bridge \
  ros-jazzy-teleop-twist-keyboard

# Python (perception용)
pip install --break-system-packages ultralytics
```

### 3. 빌드

```bash
cd ~/Documents/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. 실행

```bash
# Hospital 월드에서 자율주행
ros2 launch drobot_bringup navigation.launch.py world:=hospital_original

# RViz에서 "2D Goal Pose" 버튼으로 목표 지정
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

## 명령어 요약

| 상황 | 명령어 |
|------|--------|
| 터미널 열었을 때 | `source ~/Documents/ros2_ws/install/setup.bash` |
| 코드 수정 후 | `colcon build --symlink-install && source install/setup.bash` |
| Hospital 자율주행 | `ros2 launch drobot_bringup navigation.launch.py world:=hospital_original` |
| 키보드 조종 | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |
| 프로세스 전체 종료 | `pkill -9 -f "gz\|rviz\|nav2\|slam\|ekf\|goal"` |
