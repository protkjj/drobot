# DROBOT Description Package

**DROBOT** - Drone-Rover Hybrid Military Reconnaissance Robot  
Bulnabi Software Team / SAC2026

## 패키지 구조

```
drobot_description/
├── urdf/
│   ├── drobot.urdf.xacro      ← 메인 (모든 파일 include)
│   ├── properties.xacro        ← 변수 정의 (치수, 질량 등)
│   ├── materials.xacro         ← 색상/재질
│   ├── macros.xacro            ← 매크로 (arm, wheel, side 등)
│   ├── gazebo.xacro            ← Gazebo 전용 (폐루프, 센서, 물리)
│   └── ros2_control.xacro      ← ros2_control 인터페이스
├── meshes/                     ← STL/DAE 메시 파일 (추후)
├── worlds/                     ← Gazebo world 파일
├── launch/
│   ├── display.launch.py       ← RViz 시각화
│   └── gazebo.launch.py        ← Gazebo 시뮬레이션
├── config/
│   └── display.rviz            ← RViz 설정
├── scripts/                    ← 유틸리티 스크립트
├── CMakeLists.txt
└── package.xml
```

## Joint 트리 (한쪽 기준)

```
base_footprint
└── base_link
    ├── {L/R}_front_arm_joint [revolute, 서보1]
    │   └── {L/R}_front_arm_link
    │       └── {L/R}_side_frame_joint [fixed]
    │           └── {L/R}_side_frame_link
    │               └── {L/R}_tilt_joint [revolute, 액추에이터]
    │                   └── {L/R}_tilt_frame_link
    │                       ├── {L/R}_front_wheel_joint [continuous]
    │                       │   └── {L/R}_front_wheel_link
    │                       │       └── {L/R}_front_prop_motor [fixed]
    │                       └── {L/R}_rear_wheel_joint [continuous]
    │                           └── {L/R}_rear_wheel_link
    │                               └── {L/R}_rear_prop_motor [fixed]
    ├── {L/R}_rear_arm_joint [revolute, 서보2]
    │   └── {L/R}_rear_arm_link
    │       └── (Gazebo constraint → side_frame)
    ├── {L/R}_actuator_joint [prismatic, mimic tilt]
    │   └── {L/R}_actuator_link
    ├── lidar_joint [fixed] → lidar_link
    ├── camera_joint [fixed] → camera_link
    └── imu_joint [fixed] → imu_link
```

## 서보 동기화 (Mimic 관계)

| 서보 | Master | Slave (mimic) |
|------|--------|---------------|
| 서보1 | left_front_arm_joint | right_front_arm_joint |
| 서보2 | left_rear_arm_joint | right_rear_arm_joint |

## Closed-Loop 처리

URDF는 트리만 지원 → 폐루프를 다음과 같이 처리:

| 실제 구조 | URDF 처리 | Gazebo 보정 |
|-----------|-----------|-------------|
| rear_arm ↔ side_frame | mimic joint | `<joint>` constraint |
| actuator ↔ tilt | mimic joint | 물리 연동 |
| L/R front arm 동기화 | mimic joint | JointPositionController |
| L/R rear arm 동기화 | mimic joint | JointPositionController |

## 액추에이터 요약 (10개)

| # | 타입 | Joint | 제어 |
|---|------|-------|------|
| 1 | 서보1 | L/R front_arm_joint | position (0~90°) |
| 2 | 서보2 | L/R rear_arm_joint | position (0~90°) |
| 3-4 | 선형 액추에이터 | L/R tilt_joint | position (0~90°) |
| 5-6 | 바퀴 모터 | L/R front_wheel_joint | velocity |
| 7-10 | 프로펠러 모터 | 4x propeller_joint | velocity (추후) |

## 모드 전환 시퀀스

```
[Ground] wheels down, vertical(│)
   │
   ├── 서보1+서보2 → arm 회전 → 바퀴 위치 상승 (원호 궤적)
   │
[Mid]   wheels up, still vertical(│)
   │
   ├── 선형 액추에이터 → tilt 90° → 바퀴 수평(─)
   │
[Flight] wheels up, horizontal(─) → 프로펠러 회전
```

## 사용법

### RViz에서 확인 (joint 슬라이더)
```bash
ros2 launch drobot_description display.launch.py
```

### Gazebo 시뮬레이션
```bash
ros2 launch drobot_description gazebo.launch.py
# 또는 특정 world 지정
ros2 launch drobot_description gazebo.launch.py world:=$(ros2 pkg prefix drobot_description)/share/drobot_description/worlds/simple_room.world
```

## TODO 체크리스트

- [ ] base body 실측 치수 입력 (properties.xacro)
- [ ] arm 실측 치수 입력
- [ ] side frame 실측 치수 입력
- [ ] wheel 두께 입력 (반지름 112mm 확정)
- [ ] 각 부품 질량 입력
- [ ] mesh 파일 export (Onshape → STL)
- [ ] 센서 위치 확정
- [ ] 프로펠러 사양 확정 후 주석 해제
- [ ] Gazebo closed-loop constraint 위치 조정
- [ ] 실제 하드웨어 인터페이스 구현
