# Drobot

ROS 2 Jazzy + Gazebo Harmonic 기반 로봇 시뮬레이션

## 필수 패키지 설치

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-common \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization
```

## 빌드

```bash
cd ~/drobot
colcon build --symlink-install
source install/setup.bash
```

## 자동 지도 생성 (Auto Exploration)

한 번에 Gazebo, RViz, SLAM, Nav2, 자동탐색 모두 실행:

```bash
ros2 launch drobot_simulation auto_explore.launch.py
```

다른 월드 사용:
```bash
ros2 launch drobot_simulation auto_explore.launch.py world:=warehouse
ros2 launch drobot_simulation auto_explore.launch.py world:=office_maze
```

### 사용 가능한 월드
- `empty` - 장애물 있는 방
- `full_world` (기본) - 복합 환경
- `simple_room` - 거실
- `warehouse` - 창고
- `office_maze` - 사무실 미로
- `f1_circuit` - F1 서킷

### 실행 순서 (자동)
| 시간 | 구성 요소 |
|------|-----------|
| 0초 | Gazebo + RViz |
| 3초 | EKF Localization |
| 5초 | SLAM Toolbox |
| 8초 | Nav2 |
| 18초 | Auto Explorer |

## 수동 조작

시뮬레이션만 실행 후 키보드로 조종:
```bash
ros2 launch drobot_simulation simulation.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```