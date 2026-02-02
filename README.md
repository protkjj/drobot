# Drobot

**Drobot** = **D**rone + **Robot** 하이브리드 로봇 시뮬레이션

4바퀴 스키드 스티어 주행 + 쿼드콥터 비행이 가능한 하이브리드 로봇 프로젝트

## 프로젝트 로드맵

```
[Phase 1] 지상 주행 - 2D Navigation ✅ 완료
    ├── Nav2 + SLAM Toolbox
    ├── Frontier-based 자동 탐색
    └── 4바퀴 스키드 스티어 제어
           ↓
[Phase 2] 드론 비행 - 3D Control 🚧 진행중
    ├── PX4 SITL + Gazebo ✅
    ├── QGroundControl ✅
    ├── ROS 2 연동 (Micro XRCE-DDS) ✅
    └── 호버링/이륙/착륙 테스트
           ↓
[Phase 3] 하이브리드 - 지상+비행 전환 (예정)
    ├── 모드 전환 로직 (주행 ↔ 비행)
    ├── 3D SLAM (RTAB-Map, Octomap)
    └── 3D 경로 계획
```

## 패키지 구조

```
drobot/
├── drobot_description/   # 공용 (URDF, meshes, worlds, world_generator)
├── drobot_simulation/    # VER0 - 자동 탐색 (Frontier Exploration)
├── drobot_navigation/    # VER1 - 목표 기반 자율주행 (Goal Navigation)
└── drobot_advanced/      # VER4 - Isaac Sim + PX4 (예정)
```

## 빠른 시작

### 1. 설치 (지상 주행)
```bash
# 필수 패키지
sudo apt update && sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-common \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization

# 빌드
cd ~/drobot
colcon build --symlink-install
source install/setup.bash
```

### 2. 실행 (지상 주행)
```bash
# 터미널 열 때마다 실행
source ~/drobot/install/setup.bash

# VER1 - 목표 기반 자율주행 (추천)
ros2 launch drobot_navigation navigation.launch.py

# VER0 - 자동 탐색
ros2 launch drobot_simulation auto_explore.launch.py
```

## 사용 가능한 월드

### 기본 월드 (고정 구조)
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

### 월드 변경 예시
```bash
# VER1 - 창고에서 자율주행
ros2 launch drobot_navigation navigation.launch.py world:=warehouse

# VER1 - 랜덤 방에서 자율주행
ros2 launch drobot_navigation navigation.launch.py world:=room_generated

# VER0 - 미로 자동 탐색
ros2 launch drobot_simulation auto_explore.launch.py world:=maze_generated
```

## 목표 설정 (VER1)

RViz에서 "2D Goal Pose" 버튼 클릭 후 맵에서 위치 지정

또는 터미널에서:
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}
}"
```

## 월드 생성기 (선택사항)

새로운 커스텀 월드를 만들고 싶을 때만 사용

### CLI로 생성
```bash
cd ~/drobot/drobot_description/scripts

# 방 생성 (15x15m, 장애물 8개)
python3 world_generator.py --type room --size 15 15 --obstacles 8 --output my_room

# 미로 생성
python3 world_generator.py --type maze --size 20 20 --obstacles 6 --output my_maze

# 도로 생성 (차량 3대)
python3 world_generator.py --type road --size 20 10 --obstacles 3 --output my_road
```

### 생성 후 실행
```bash
cd ~/drobot && colcon build --symlink-install
source install/setup.bash
ros2 launch drobot_navigation navigation.launch.py world:=my_room
```

### Python API
```python
from world_generator import WorldGenerator

gen = WorldGenerator(
    name="custom_world",
    room_size=(15, 15),
    spawn_point=(0, 0),
    min_obstacle_gap=4.0,  # 장애물 간격 (로봇 통과용)
    seed=42                # 랜덤 시드 (재현성)
)

gen.add_walls(doors=["south"])
gen.add_random_obstacles(count=10)
gen.add_moving_obstacle(path=[(2, 0), (2, 4)], speed=0.5)
gen.save("custom_world.sdf")
```

## 수동 조작

```bash
# 시뮬레이션만 실행
ros2 launch drobot_simulation simulation.launch.py

# 다른 터미널에서 키보드 조종
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## PX4 드론 비행 (Phase 2)

### PX4 설치
```bash
# 의존성
sudo apt install -y git make cmake python3-pip python3-jinja2 python3-empy \
  python3-toml python3-numpy python3-packaging python3-jsonschema libfuse2 \
  gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
  libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev

# PX4 클론 및 설치
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
sudo reboot

# 첫 빌드 (10-20분 소요)
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

### QGroundControl 설치
```bash
# 사전 설정
sudo usermod -aG dialout "$(id -un)"
sudo systemctl mask --now ModemManager.service

# 다운로드 (브라우저에서)
# https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html

# 실행 권한
chmod +x ~/Downloads/QGroundControl-x86_64.AppImage
```

### Micro XRCE-DDS Agent 설치 (ROS 2 브릿지)
```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### px4_msgs 설치
```bash
cd ~/drobot
git clone https://github.com/PX4/px4_msgs.git -b release/1.15
colcon build --packages-select px4_msgs
source install/setup.bash
```

### PX4 드론 실행 (터미널 4개)

| 터미널 | 명령어 |
|--------|--------|
| T1 | `cd ~/PX4-Autopilot && make px4_sitl gz_x500` |
| T2 | `MicroXRCEAgent udp4 -p 8888` |
| T3 | `~/Downloads/QGroundControl-x86_64.AppImage` |
| T4 | `source ~/drobot/install/setup.bash && ros2 topic list` |

### 이륙 테스트
```bash
# PX4 콘솔(pxh>)에서
commander takeoff

# 착륙
commander land
```

### 유용한 PX4 명령어 (pxh>)

| 명령어 | 설명 |
|--------|------|
| `commander arm` | 시동 |
| `commander disarm` | 시동 끄기 |
| `commander takeoff` | 이륙 |
| `commander land` | 착륙 |
| `commander mode posctl` | 위치 제어 모드 |
| `param set COM_RCL_EXCEPT 4` | RC 없이 Offboard 허용 |
| `param set NAV_DLL_ACT 0` | GCS 끊김 failsafe 비활성화 |
| `param save` | 파라미터 저장 |

---

## 명령어 요약

| 상황 | 명령어 |
|------|--------|
| 터미널 열었을 때 | `source ~/drobot/install/setup.bash` |
| 코드 수정 후 | `colcon build && source install/setup.bash` |
| VER1 실행 | `ros2 launch drobot_navigation navigation.launch.py` |
| VER0 실행 | `ros2 launch drobot_simulation auto_explore.launch.py` |
| 월드 변경 | `... world:=warehouse` |
| 키보드 조종 | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |
| 프로세스 종료 | `pkill -9 -f "gz\|rviz"` |

## 문서

- [CLAUDE.md](CLAUDE.md) - 상세 개발 메모 및 파라미터 설정
