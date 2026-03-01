# DROBOT

드론-로버 하이브리드 변형 로봇

## 초기 세팅

### Docker (권장, Ubuntu)

Docker를 사용하면 의존성 설치 없이 동일한 환경에서 실행됩니다.
Gazebo/RViz 등 GUI가 필요하므로 **Ubuntu에서 사용**을 권장합니다. (Mac에서는 GUI 불가)

```bash
git clone https://github.com/protkjj/drobot.git
cd drobot
docker compose up --build
```

### 로컬 설치 (Ubuntu 24.04 + ROS2 Jazzy)

```bash
cd ~/Desktop
git clone https://github.com/protkjj/drobot.git
cd drobot
bash setup.sh
```

비밀번호 한 번 입력 후 자동으로 의존성 설치 및 ROS2 빌드까지 완료됩니다.

## 실행 방법

### Docker 실행

```bash
cd drobot

# DROBOT만 실행
docker compose up drobot

# PX4 + DDS Agent + DROBOT 전부 실행
docker compose up

# DROBOT 컨테이너 접속
docker exec -it drobot bash
ros2 launch drobot_bringup navigation.launch.py world:=hospital
```

### 로컬 실행

#### UI 런처 (권장)

```bash
cd ~/Desktop/drobot/ros2_ws && source install/setup.bash
ros2 launch drobot_bringup ui.launch.py
```

Tkinter UI에서 월드/목표/옵션을 선택하면 Terminator 분할 창으로 필요한 노드들이 자동 실행됩니다.

#### 수동 실행

```bash
cd ~/Desktop/drobot/ros2_ws && source install/setup.bash

# Navigation (Gazebo + SLAM + Nav2 + Goal Navigator)
ros2 launch drobot_bringup navigation.launch.py world:=<월드이름>

# Teleop (별도 터미널)
ros2 run drobot_controller teleop_keyboard

# Perception (별도 터미널, 선택)
ros2 launch drobot_bringup perception.launch.py
```

### Teleop 키 조작

```
이동:                   홀로노믹(shift):
   u    i    o             U    I    O
   j    k    l             J    K    L
   m    ,    .             M    <    >

t/b : 상승/하강          q/z : 전체 속도 증감
w/x : 직진 속도 증감     e/c : 회전 속도 증감

Arm:
   1 : fold (지상 모드)
   2 : unfold (비행 모드)
```

## 코드 수정 후 반영

### Docker

소스코드가 volume 마운트되어 있으므로 로컬에서 코드를 수정하면 컨테이너에 바로 반영됩니다.

```bash
# 최초 1회 (symlink-install로 빌드하면 이후 Python 코드는 빌드 불필요)
docker exec -it drobot bash
cd /root/ros2_ws
colcon build --symlink-install

# 이후 Python 코드 수정 시 → 빌드 없이 바로 실행 가능
# C++ 코드 수정 시 → colcon build 필요
```

### 로컬

```bash
cd ~/Desktop/drobot/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 코드 업데이트 받기 (git pull)

### Docker

```bash
cd drobot
git pull
# 소스코드만 바뀐 경우 → colcon build만 하면 됨
docker exec -it drobot bash
cd /root/ros2_ws && colcon build

# Dockerfile이 바뀐 경우 → 이미지 재빌드 필요
docker compose up --build
```

### 로컬

```bash
cd ~/Desktop/drobot
git pull
cd ros2_ws && colcon build
```

## 폴더 구조

```
drobot/
├── Dockerfile               # DROBOT Docker 이미지
├── Dockerfile.px4           # PX4 SITL Docker 이미지
├── docker-compose.yml       # Docker Compose 구성
├── entrypoint.sh            # 컨테이너 엔트리포인트
├── .dockerignore            # Docker 빌드 제외 파일
├── setup.sh                 # 로컬 환경 세팅 스크립트
├── README.md
└── ros2_ws/
    └── src/
        ├── drobot_description/  # URDF, 메시, 월드, Gazebo 플러그인
        ├── drobot_bringup/      # Launch 파일, Nav2/SLAM/EKF 설정
        ├── drobot_controller/   # teleop_keyboard (수동 조작)
        ├── drobot_scan_2d/      # goal_navigator, 2D 스캔 기반 네비게이션
        ├── drobot_scan_3d/      # 3D 스캔 처리
        ├── drobot_simulator/    # 시뮬레이션 유틸
        ├── drobot_strategy/     # 전략/행동 로직
        └── perception/          # YOLO 기반 콘 인식 (카메라 + depth)
```

## 주요 설정 파일

| 파일 | 설명 |
|------|------|
| `drobot_bringup/config/navigation/nav2_params.yaml` | Nav2 파라미터 (플래너, costmap, 속도 제한) |
| `drobot_bringup/config/navigation/navigate_with_replanning.xml` | Behavior Tree (경로 추종 + 복구 동작) |
| `drobot_bringup/config/navigation/rules.yaml` | 자율주행 규칙 |
| `drobot_bringup/config/common/slam_params.yaml` | SLAM 설정 |
| `drobot_bringup/config/common/ekf.yaml` | EKF 로컬라이제이션 |
| `drobot_bringup/config/spawn_positions.yaml` | 월드별 로봇 스폰 위치 |
| `drobot_description/urdf/drobot.urdf.xacro` | 로봇 URDF (관절, 센서) |
| `drobot_description/urdf/gazebo.xacro` | Gazebo 플러그인 (PID, 센서, Diff Drive) |
| `drobot_description/scripts/world_generator.py` | 월드 생성기 (콘 배치) |
