# Drobot 프로젝트 메모

## 프로젝트 개요
- ROS 2 Jazzy + Gazebo Harmonic 기반 로봇 시뮬레이션
- 4바퀴 스키드 스티어 + 쿼드콥터 프로펠러 로봇
- 자율 탐색 (Frontier-based Exploration) + SLAM

## 현재 상태
- [x] 기본 URDF 완성
- [x] Gazebo 플러그인 설정 완료 (DiffDrive, LiDAR, Camera, IMU)
- [x] SLAM Toolbox 연동
- [x] Nav2 Navigation Stack 연동
- [x] 자동 탐색 (auto_explorer.py) 구현
- [x] STL 메시 적용 (전체 모델 1개, drobot v0.2.STL)
- [ ] STL 부위별 분리 (바퀴, 프로펠러, 본체 등)

## 주요 설정값

### 로봇 파라미터
| 항목 | 값 |
|------|-----|
| robot_radius | 0.22m |
| inflation_radius | 0.35m |
| max_vel_x | 0.22 m/s |
| max_vel_theta | 0.8 rad/s |
| LiDAR 범위 | 12.0m |

### 프레임 구조
```
map → odom → base_footprint → base_link → lidar_link/camera_link/imu_link
 ↑      ↑
SLAM   EKF (odom→base_footprint TF 발행)
```

### TF 설정
- EKF: `publish_tf: true` (odom → base_footprint)
- Gazebo DiffDrive: `publish_odom_tf: false` (중복 방지)

## TODO
1. **STL 부위별 분리**
   - SolidWorks에서 파트별로 export
   - 바퀴 4개, 프로펠러 4개, 본체 분리
   - 각 링크에 개별 STL 적용

2. **탐색 성능 개선**
   - "Start occupied" 오류 빈도 줄이기
   - 성공률 향상 (현재 ~5%)

3. **추후 확장**
   - 음성/텍스트 명령 (LLM 연동)
   - 객체 인식
   - Waypoint 저장/이동

## 주요 명령어

### 빌드
```bash
cd ~/drobot && colcon build --packages-select drobot_simulation && source install/setup.bash
```

### 자동 탐색 실행
```bash
# Empty (기본)
ros2 launch drobot_simulation auto_explore.launch.py

# Warehouse
ros2 launch drobot_simulation auto_explore.launch.py world:=warehouse

# F1 Circuit
ros2 launch drobot_simulation auto_explore.launch.py world:=f1_circuit

# Office Maze
ros2 launch drobot_simulation auto_explore.launch.py world:=office_maze

# 시작 위치 지정
ros2 launch drobot_simulation auto_explore.launch.py world:=warehouse spawn_x:=1.0 spawn_y:=1.0
```

### 기타
```bash
# 시뮬레이션만 (탐색 없이)
ros2 launch drobot_simulation simulation.launch.py

# 키보드 조종
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 파일 구조
```
drobot/
├── CLAUDE.md                    # 이 파일
├── files/                       # SolidWorks 원본 파일
│   └── drobot v0.2.STL
└── drobot_simulation/
    ├── urdf/drobot.urdf.xacro   # 로봇 모델
    ├── meshes/drobot_body.stl   # STL 메시
    ├── worlds/                   # Gazebo 월드들
    ├── launch/
    │   ├── simulation.launch.py      # 시뮬레이션
    │   ├── auto_explore.launch.py    # 자동 탐색
    │   └── localization.launch.py    # EKF
    ├── config/
    │   ├── nav2_params.yaml     # Nav2 설정
    │   ├── slam_params.yaml     # SLAM 설정
    │   ├── ekf.yaml             # EKF 설정
    │   └── ros_gz_bridge.yaml   # Gazebo 브릿지
    └── drobot_simulation/
        └── auto_explorer.py     # 자동 탐색 노드
```

## 알려진 이슈
1. **"Start occupied" 오류**: Costmap에서 로봇 위치가 장애물로 인식됨
   - 해결: costmap 클리어 + 후진 recovery

2. **STL 20MB**: 파일이 커서 Gazebo 로딩 느림
   - 해결 예정: 폴리곤 수 줄이기 또는 부위별 분리

3. **탐색 성공률 낮음**: 목표 도달 성공률 ~5%
   - Coverage는 증가하지만 많은 시도 필요
