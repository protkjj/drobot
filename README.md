# Drobot

**Drobot** = **D**rone + **Robot** 하이브리드 로봇 시뮬레이션

4바퀴 스키드 스티어 주행 + 쿼드콥터 비행이 가능한 하이브리드 로봇 프로젝트

## 프로젝트 목표

```
[Phase 1] 지상 주행 (현재) ──→ [Phase 2] 드론 비행 ──→ [Phase 3] 하이브리드
   Nav2 + SLAM                  PX4 + MAVROS            모드 전환 + 3D SLAM
```

## 기술 스택

| 구성 요소 | 현재 (Phase 1) | 예정 (Phase 2+) |
|----------|---------------|-----------------|
| 시뮬레이터 | Gazebo Harmonic | + Isaac Sim (선택) |
| ROS | ROS 2 Jazzy | 동일 |
| SLAM | SLAM Toolbox (2D) | RTAB-Map (3D) |
| Navigation | Nav2 | + PX4/MAVROS |
| 센서 | 2D LiDAR, Camera, IMU | + 3D LiDAR |

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

## Phase 2 준비 (드론 비행)

PX4 SITL 설치:
```bash
# PX4 Autopilot
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh

# MAVROS
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras
```

## 시뮬레이터 비교

| 시뮬레이터 | 장점 | 단점 |
|-----------|------|------|
| **Gazebo** (현재) | PX4 공식 지원, 커뮤니티 큼 | 그래픽 평범 |
| **Isaac Sim** | 포토리얼리스틱, AI 학습 | RTX 3070+, 설정 복잡 |

## 문서

- [CLAUDE.md](CLAUDE.md) - 상세 개발 메모 및 설정값