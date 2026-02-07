# Drobot 프로젝트 메모

## 프로젝트 개요
- **Drobot** = **D**rone + **Robot** 하이브리드
- ROS 2 Jazzy + Gazebo Harmonic 기반 시뮬레이션
- 4바퀴 스키드 스티어 + 쿼드콥터 프로펠러 로봇
- 자율 탐색 (Frontier-based Exploration) + SLAM

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
    ├── Micro XRCE-DDS Agent (ROS 2 브릿지) ✅
    ├── px4_msgs 패키지 ✅
    └── 호버링/이륙/착륙 테스트
           ↓
[Phase 3] 하이브리드 - 지상+비행 전환 (최종 목표)
    ├── 모드 전환 로직 (주행 ↔ 비행)
    ├── 3D SLAM (RTAB-Map, Octomap)
    └── 3D 경로 계획
```

## 기술 스택 비교

### 시뮬레이터 선택
| 시뮬레이터 | 장점 | 단점 | 용도 |
|-----------|------|------|------|
| **Gazebo** (현재) | PX4 공식 지원, 문서 풍부 | 그래픽 평범 | 프로토타이핑 |
| **Isaac Sim** | 포토리얼리스틱, AI 학습 최적 | 높은 사양, 설정 복잡 | 최종 단계 |
| **AirSim** | 드론 특화, 설정 쉬움 | 개발 중단 (2022) | 비추천 |

### Phase별 필요 기술
| Phase | 센서 | SLAM | 제어 | 난이도 |
|-------|------|------|------|--------|
| 1 (지상) | 2D LiDAR | SLAM Toolbox | Nav2 DWB | ★★☆ |
| 2 (비행) | IMU, Barometer | - | PX4/MAVROS | ★★★ |
| 3 (하이브리드) | 3D LiDAR | RTAB-Map | Custom | ★★★★ |

### Isaac Sim 권장 사양
- **GPU**: RTX 3070+ (VRAM 8GB+)
- **RAM**: 32GB+
- **CPU**: 8코어+
- **저장소**: NVMe SSD

## 현재 상태
- [x] 기본 URDF 완성
- [x] Gazebo 플러그인 설정 완료 (DiffDrive, LiDAR, Camera, IMU)
- [x] SLAM Toolbox 연동
- [x] Nav2 Navigation Stack 연동
- [x] 자동 탐색 (auto_explorer.py) 구현
- [x] STL 메시 적용 (전체 모델 1개, drobot_body.stl)
- [x] STL 원점 보정 (TF 프레임 정렬)
- [x] "Start occupied" 오류 해결
- [ ] STL 부위별 분리 (바퀴, 프로펠러, 본체 등)

## 주요 설정값

### 로봇 파라미터 (nav2_params.yaml)
| 항목 | 값 | 설명 |
|------|-----|------|
| robot_radius | 0.05m | costmap 충돌 범위 (최소화) |
| inflation_radius | 0.15m | 장애물 확장 범위 |
| max_vel_x | 0.22 m/s | 최대 전진 속도 |
| max_vel_theta | 0.8 rad/s | 최대 회전 속도 |
| obstacle_min_range | 0.35m | 최소 장애물 감지 거리 |
| raytrace_min_range | 0.35m | 최소 레이캐스트 거리 |
| laser_min_range | 0.35m | AMCL 최소 레이저 거리 |

### LiDAR 설정 (drobot.urdf.xacro)
| 항목 | 값 | 설명 |
|------|-----|------|
| min range | 0.35m | 로봇 자체 감지 방지 |
| max range | 12.0m | 최대 감지 거리 |
| samples | 360 | 수평 샘플 수 |

### STL 메시 원점 보정 (drobot.urdf.xacro)
```xml
<visual name="body_mesh">
  <geometry>
    <mesh filename="package://drobot_simulation/meshes/drobot_body.stl" scale="0.001 0.001 0.001"/>
  </geometry>
  <!-- STL 원점 보정: 로봇 중심으로 이동 -->
  <origin xyz="-0.18 0.2 -0.05" rpy="1.5708 0 0"/>
  <material name="orange"/>
</visual>
```
- X: -0.18 (뒤로)
- Y: 0.2 (왼쪽으로)
- Z: -0.05 (아래로)
- rpy: 1.5708 0 0 (X축 90도 회전, SolidWorks→ROS 좌표계 변환)

### 맵별 스폰 위치 (auto_explore.launch.py)
| 월드 | spawn_x | spawn_y | 설명 |
|------|---------|---------|------|
| full_world | 0.0 | 0.0 | 기본 |
| empty | 0.0 | 0.0 | 빈 맵 |
| simple_room | 0.0 | 0.0 | 단순 방 |
| warehouse | 0.0 | -3.0 | 선반 아래쪽 빈 공간 |
| f1_circuit | 0.0 | 3.5 | 트랙 위 (빨간/파란 경계 사이) |
| office_maze | -4.0 | -3.0 | 미로 코너 빈 공간 |

### 프레임 구조
```
map → odom → base_footprint → base_link → lidar_link/camera_link/imu_link
 ↑      ↑
SLAM   EKF (odom→base_footprint TF 발행)
```

### TF 설정
- EKF: `publish_tf: true` (odom → base_footprint)
- Gazebo DiffDrive: `publish_odom_tf: false` (중복 방지)

## 해결된 이슈

### "Start occupied" 오류 (2026-01-31 해결)
**증상**: Costmap에서 로봇 위치가 장애물로 인식되어 네비게이션 실패

**원인**:
1. STL 메시 원점이 로봇 중심이 아닌 바퀴 끝부분에 있어서 TF 프레임과 시각적 메시가 어긋남
2. LiDAR가 로봇 자체(다리/바퀴)를 장애물로 감지

**해결책**:
1. STL visual origin 보정: `xyz="-0.18 0.2 -0.05"`
2. LiDAR 최소 감지 거리 증가: 0.1m → 0.35m
3. Costmap 파라미터 축소:
   - robot_radius: 0.22 → 0.05
   - inflation_radius: 0.35 → 0.15
   - obstacle_min_range: 0.0 → 0.35
   - raytrace_min_range: 0.0 → 0.35

## TODO
정확한 구분                                                                                                                                                           
  ┌────────────────────┬───────────────────┬─────────────────────────────────┐                                                                                          
  │        역할        │       도구        │              설명               │                                                                                          
  ├────────────────────┼───────────────────┼─────────────────────────────────┤                                                                                          
  │ 로봇 시뮬레이터    │ Isaac Sim         │ 물리엔진 + 렌더링 + 센서        │                                                                                          
  ├────────────────────┼───────────────────┼─────────────────────────────────┤                                                                                          
  │ AI 학습 프레임워크 │ Isaac Lab         │ RL 학습 (Isaac Sim 위에서 실행) │                                                                                          
  ├────────────────────┼───────────────────┼─────────────────────────────────┤                                                                                          
  │ 드론 오토파일럿    │ PX4               │ 비행 제어 펌웨어                │                                                                                          
  ├────────────────────┼───────────────────┼─────────────────────────────────┤                                                                                          
  │ 드론 시뮬레이션    │ Gazebo + PX4 SITL │ PX4를 시뮬레이션에서 실행       │                                                                                          
  └────────────────────┴───────────────────┴────────────────────────────

### Phase 1 (지상 주행) - 현재
- [x] 기본 자율 탐색 구현
- [x] SLAM + Nav2 연동
- [ ] STL 부위별 분리 (바퀴 회전 시각화)
- [ ] 탐색 성공률 개선
- [ ] Waypoint 저장/이동

### Phase 2 (드론 비행) - 진행중
- [x] PX4 SITL 설치 및 연동
- [x] QGroundControl 설치
- [x] Micro XRCE-DDS Agent 설치 (ROS 2 브릿지)
- [x] px4_msgs 패키지 설치
- [ ] 호버링/이륙/착륙 테스트
- [ ] drobot에 PX4 플러그인 연동
- [ ] 프로펠러 제어 구현

### Phase 3 (하이브리드) - 최종
- [ ] 주행 ↔ 비행 모드 전환 로직
- [ ] 3D LiDAR 추가 (Velodyne/Ouster 시뮬레이션)
- [ ] RTAB-Map 3D SLAM 연동
- [ ] Octomap 3D 경로 계획
- [ ] Isaac Sim 마이그레이션 (선택)

## PX4 드론 비행 설정

### 설치 완료 항목
- PX4-Autopilot: `~/PX4-Autopilot`
- QGroundControl: `~/Downloads/QGroundControl-x86_64.AppImage`
- Micro XRCE-DDS Agent: 시스템 설치 완료
- px4_msgs: `~/drobot/px4_msgs`

### PX4 실행 순서 (터미널 4개)
| 순서 | 터미널 | 명령어 |
|------|--------|--------|
| 1 | T1 | `cd ~/PX4-Autopilot && make px4_sitl gz_x500` |
| 2 | T2 | `MicroXRCEAgent udp4 -p 8888` |
| 3 | T3 | `~/Downloads/QGroundControl-x86_64.AppImage` |
| 4 | T4 | `source ~/drobot/install/setup.bash && ros2 topic list` |

### PX4 콘솔 명령어 (pxh>)
| 명령어 | 설명 |
|--------|------|
| `commander arm` | 시동 |
| `commander disarm` | 시동 끄기 |
| `commander takeoff` | 이륙 |
| `commander land` | 착륙 |
| `commander mode posctl` | 위치 제어 모드 |
| `listener vehicle_local_position` | 위치 확인 |
| `param set COM_RCL_EXCEPT 4` | RC 없이 Offboard 허용 |
| `param set NAV_RCL_ACT 0` | RC 끊김 failsafe 비활성화 |
| `param set NAV_DLL_ACT 0` | GCS 끊김 failsafe 비활성화 |
| `param save` | 파라미터 저장 |

### ROS 2 PX4 토픽
```bash
# 토픽 목록
ros2 topic list | grep fmu

# 위치 확인
ros2 topic echo /fmu/out/vehicle_local_position

# 상태 확인
ros2 topic echo /fmu/out/vehicle_status
```

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

# 좀비 프로세스 정리
pkill -9 -f "rviz|gz"
```

## 파일 구조
```
drobot/
├── CLAUDE.md                    # 이 파일
├── README.md                    # 사용법 가이드
├── files/                       # SolidWorks 원본 파일
│   └── drobot v0.2.STL
├── px4_msgs/                    # PX4 ROS 2 메시지 (외부 패키지)
├── drobot_description/          # 공용 리소스
│   ├── urdf/drobot.urdf.xacro   # 로봇 모델
│   ├── meshes/
│   │   └── drobot_body.stl      # 전체 STL (20MB)
│   └── worlds/                   # Gazebo 월드들
│       ├── empty.sdf, full_world.sdf, warehouse.sdf
│       ├── f1_circuit.sdf, office_maze.sdf
│       └── generated/            # 자동 생성 월드
├── drobot_simulation/           # VER0 자동 탐색
│   ├── launch/
│   │   ├── auto_explore.launch.py    # 자동 탐색 (메인)
│   │   ├── simulation.launch.py      # 시뮬레이션만
│   │   └── localization.launch.py    # EKF
│   ├── config/
│   │   ├── nav2_params.yaml     # Nav2 설정 (탐색용)
│   │   ├── slam_params.yaml     # SLAM 설정
│   │   └── ekf.yaml             # EKF 설정
│   └── drobot_simulation/
│       └── auto_explorer.py     # 자동 탐색 노드
├── drobot_navigation/           # VER1 목표 이동
│   ├── launch/navigation.launch.py
│   ├── config/nav2_params.yaml  # Nav2 설정 (이동용)
│   └── drobot_navigation/
│       └── goal_navigator.py    # 목표 네비게이션 노드
```

## 알려진 이슈
1. **STL 20MB**: drobot_body.stl 파일이 커서 Gazebo 로딩 느림
   - 해결 예정: 폴리곤 수 줄이기

2. **바퀴 회전 시각화 안됨**: 통째 STL 사용으로 바퀴가 굴러가는 것처럼 보이지 않음
   - 해결 예정: SolidWorks에서 부위별 STL 재추출 후 각 링크에 적용

3. **긴 경로 탐색 실패**: 먼 목표로 이동 시 "No progress" 발생
   - 원인: 좁은 통로나 복잡한 경로에서 막힘
   - Coverage는 계속 증가하므로 큰 문제는 아님
