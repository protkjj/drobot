# Drobot 프로젝트 메모

## 프로젝트 개요
- **Drobot** = **D**rone + **Robot** 하이브리드
- ROS 2 Jazzy + Gazebo Harmonic 기반 시뮬레이션
- 4바퀴 스키드 스티어 + 쿼드콥터 프로펠러 로봇

## 버전 구조

```
drobot/
├── drobot_simulation/     # VER0 - 프론티어 탐색 (Frontier Exploration)
├── drobot_navigation/     # VER1 - 자율주행 + 규칙 엔진 (Goal Navigation)
└── drobot_advanced/       # VER4 - Isaac Sim/Lab + PX4 통합 (예정)
```

| 버전 | 패키지 | 목적 | 상태 |
|------|--------|------|------|
| VER0 | drobot_simulation | 프론티어 기반 지도 탐색 | ✅ 완료 |
| VER1 | drobot_navigation | 목표 기반 자율주행 + 규칙 | 🔄 개발중 |
| VER4 | drobot_advanced | Isaac Sim + PX4 통합 | 📋 계획 |

## 프로젝트 로드맵

```
[VER0] 프론티어 탐색 ✅
    └── 지도 작성을 위한 자동 탐색
           ↓
[VER1] 자율주행 🔄 (현재)
    ├── 사용자 지정 목표로 이동
    ├── 규칙 엔진 (금지구역, 속도제한, 정지규칙)
    └── 깔끔한 파라미터 정리
           ↓
[VER4] 고급 통합 📋
    ├── Isaac Sim (포토리얼리스틱 시뮬레이션)
    ├── Isaac Lab (강화학습 훈련)
    ├── PX4 (드론 비행 제어)
    └── 하이브리드 지상/비행 전환
```

## VER0 - 프론티어 탐색 (drobot_simulation)

### 특징
- Frontier-based Exploration
- SLAM Toolbox + Nav2
- 자동으로 미지 영역 탐색

### 실행
```bash
ros2 launch drobot_simulation auto_explore.launch.py
ros2 launch drobot_simulation auto_explore.launch.py world:=warehouse
```

### 알려진 이슈
- 프론티어 알고리즘 특성상 벽 근처 목표 선택 경향
- DWB Local Planner가 좁은 통로에서 막힘
- 파라미터 튜닝이 복잡해짐 → VER1에서 정리

## VER1 - 자율주행 (drobot_navigation)

### 특징
- 사용자 지정 목표 (RViz 클릭 또는 좌표)
- 규칙 엔진 (`rules.yaml`)
  - 금지 구역 (forbidden_zones)
  - 속도 제한 (speed_limit_zones)
  - 정지 규칙 (stop_rules)
- 정리된 Nav2 파라미터

### 실행
```bash
ros2 launch drobot_navigation navigation.launch.py
ros2 launch drobot_navigation navigation.launch.py world:=empty
```

### 목표 설정
```bash
# RViz: "2D Goal Pose" 버튼 클릭 후 맵에서 위치 지정

# 또는 터미널:
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}
}"
```

### 파일 구조
```
drobot_navigation/
├── config/
│   ├── nav2_params.yaml    # 정리된 Nav2 파라미터
│   └── rules.yaml          # 주행 규칙 정의
├── launch/
│   └── navigation.launch.py
└── drobot_navigation/
    ├── goal_navigator.py   # 메인 노드
    ├── config.py           # 설정 상수
    └── rules/
        └── engine.py       # 규칙 엔진
```

## VER4 - 고급 통합 (drobot_advanced)

### 계획된 기능
- **Isaac Sim**: 포토리얼리스틱 렌더링, 도메인 랜덤화
- **Isaac Lab**: 강화학습 환경, 정책 훈련
- **PX4**: 드론 비행 제어, MAVROS 연동
- **하이브리드**: 지상/비행 모드 전환

### 필요 사양 (Isaac Sim)
- GPU: RTX 3070+ (VRAM 8GB+)
- RAM: 32GB+
- CPU: 8코어+
- 저장소: NVMe SSD

### 로드맵
```
drobot_advanced/config/roadmap.yaml 참조
```

## 기술 스택

| 구분 | VER0/VER1 | VER4 |
|------|-----------|------|
| 시뮬레이터 | Gazebo Harmonic | Isaac Sim |
| SLAM | SLAM Toolbox (2D) | RTAB-Map (3D) |
| 플래너 | Nav2 SmacPlanner2D | Octomap + Custom |
| 학습 | - | Isaac Lab (RL) |
| 드론 | - | PX4 + MAVROS |

## 주요 명령어

### 빌드
```bash
# 전체 빌드
cd ~/drobot && colcon build && source install/setup.bash

# 특정 패키지
colcon build --packages-select drobot_simulation
colcon build --packages-select drobot_navigation
```

### 실행
```bash
# VER0 - 프론티어 탐색
ros2 launch drobot_simulation auto_explore.launch.py

# VER1 - 자율주행
ros2 launch drobot_navigation navigation.launch.py

# 키보드 조종
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 프로세스 정리
pkill -9 -f "gz|rviz"
```

## 파일 구조 (전체)
```
drobot/
├── CLAUDE.md                    # 이 파일
├── TODO.md                      # 할 일 목록
├── files/                       # SolidWorks 원본 파일
│
├── drobot_simulation/           # VER0
│   ├── urdf/drobot.urdf.xacro
│   ├── meshes/
│   ├── worlds/
│   ├── launch/
│   ├── config/
│   └── drobot_simulation/
│       ├── auto_explorer.py
│       ├── config.py
│       ├── types.py
│       ├── utils.py
│       ├── frontier/
│       └── navigation/
│
├── drobot_navigation/           # VER1
│   ├── launch/navigation.launch.py
│   ├── config/
│   │   ├── nav2_params.yaml
│   │   └── rules.yaml
│   └── drobot_navigation/
│       ├── goal_navigator.py
│       ├── config.py
│       └── rules/engine.py
│
└── drobot_advanced/             # VER4 (Placeholder)
    ├── config/roadmap.yaml
    └── drobot_advanced/
        ├── isaac/
        └── px4/
```

## 로봇 파라미터

### 물리적 특성
| 항목 | 값 |
|------|-----|
| robot_radius | 0.12m |
| max_vel_x | 0.22 m/s |
| max_vel_theta | 0.8 rad/s |
| LiDAR min_range | 0.35m |
| LiDAR max_range | 12.0m |

### Nav2 핵심 파라미터 (VER1)
| 항목 | 값 | 설명 |
|------|-----|------|
| inflation_radius (global) | 0.55m | 경로 생성용 (크게) |
| inflation_radius (local) | 0.25m | 경로 추종용 (작게) |
| cost_scaling_factor | 1.5~3.0 | 비용 감소율 |
| cost_penalty | 3.0 | costmap 비용 페널티 |
| xy_goal_tolerance | 0.25m | 목표 도달 허용 오차 |

## 네비게이션 전략 (VER1)

### 1. 이중 Inflation 전략 ⭐
**문제**: 경로 생성 시 벽 근처로 가면 로컬 플래너가 막힘
**해결**: Global과 Local costmap에 다른 inflation 사용

```
┌─────────────────────────────────────────────────────────┐
│  Global Costmap (경로 생성)     Local Costmap (경로 추종)  │
│  ├── inflation_radius: 0.55    ├── inflation_radius: 0.25 │
│  └── cost_scaling_factor: 1.5  └── cost_scaling_factor: 3.0│
└─────────────────────────────────────────────────────────┘

동작 원리:
1. Global: 큰 inflation → 벽에서 멀리 경로 생성
   ████████████████
   ░░░░░░░░░░░░░░░░  ← 넓은 inflation (회피)
   ○ ─────────────→ ★
   ░░░░░░░░░░░░░░░░
   ████████████████

2. Local: 작은 inflation → 여유 있게 경로 추종
   ████████████████
   ░░░░            ← 좁은 inflation (통과 가능)
   ○ ═══════════════→ ★
   ░░░░
   ████████████████
```

### 2. 4륜 스키드 스티어 특성
```yaml
# nav2_params.yaml - DWB 설정
min_vel_x: -0.15      # 후진 가능
max_vel_x: 0.22       # 전진 속도
min_vel_y: 0.0        # 횡이동 불가 (스키드스티어)
max_vel_y: 0.0        # 횡이동 불가
max_vel_theta: 1.0    # 제자리 회전 가능
```

**스키드 스티어 로봇 움직임:**
```
전진 ✅   후진 ✅   좌우이동 ❌   제자리 회전 ✅
  ↑        ↓         ←→            ↻
```

### 3. 플래너 선택
| 플래너 | 특징 | 용도 |
|--------|------|------|
| NavfnPlanner | 최단 경로, 빠름 | 열린 공간 |
| SmacPlanner2D | 비용 고려, 균형 | 일반 환경 |
| **SmacPlannerHybrid** | 비홀로노믹, 안전 | **VER1 사용** |

**SmacPlannerHybrid 핵심 파라미터:**
```yaml
cost_penalty: 3.0           # costmap 비용 배율 (높을수록 안전 경로)
cost_travel_multiplier: 15.0 # 경로 비용 가중치
motion_model: "DUBIN"       # 스키드스티어에 적합
minimum_turning_radius: 0.2  # 최소 회전 반경
```

### 4. 안전 vs 최단 경로 트레이드오프
```
cost_penalty 낮음 (1.0)     cost_penalty 높음 (5.0)
┌──────────────┐            ┌──────────────┐
│ ████         │            │ ████         │
│ ████ ○──→★   │            │ ████         │
│ ████         │            │ ████ ○       │
│              │            │      ╲       │
│              │            │       ╲──→ ★ │
└──────────────┘            └──────────────┘
   최단 경로                    안전 경로
   (벽 근처)                   (벽에서 멀리)
```

## 해결된 이슈

### "Start occupied" 오류 (2026-01-31)
- **원인**: STL 원점 오프셋 + LiDAR 자체 감지
- **해결**: STL origin 보정 + LiDAR min_range 0.35m

### 프론티어 탐색 한계 (2026-02-01)
- **원인**: 프론티어 알고리즘이 벽 근처를 "가치 높음"으로 판단
- **해결**: VER1 자율주행 시스템으로 전환
