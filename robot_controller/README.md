# Robot Controller ROS 2 Package

Caltech M4 영감을 받은 지상+비행 하이브리드 로봇 제어 패키지

## 프로젝트 구조

```
robot_controller/
├── robot_controller/                  # Python 노드
│   ├── __init__.py
│   ├── mode_manager_node.py        # 모드 전환 상태머신
│   ├── wheel_controller_node.py    # 바퀴 제어 (Differential Drive)
│   ├── px4_interface_node.py       # PX4 비행 제어
│   ├── commander_node.py           # 통합 키보드 조작 (지상+비행)
│   └── teleop_keyboard_node.py     # RC카 키보드 조종 (지상 전용)
├── launch/
│   ├── bringup.launch.py           # 전체 시스템 실행
│   ├── simulation.launch.py        # 시뮬레이션용
│   └── teleop.launch.py            # RC카 키보드 조작용
├── config/
│   └── params.yaml                 # 파라미터 설정
├── resource/
│   └── robot_controller
├── package.xml
├── setup.py
└── setup.cfg
```

## 노드 설명

| 노드 | 설명 |
|------|------|
| `mode_manager_node` | GROUND/FLIGHT/TRANSITION 모드 상태머신 관리 |
| `wheel_controller_node` | `/cmd_vel` → 바퀴 모터 PWM 변환 (Differential Drive) |
| `px4_interface_node` | PX4와 통신하여 비행 제어 |
| `commander_node` | 지상+비행 통합 키보드 조작 |
| `teleop_keyboard_node` | RC카 전용 키보드 조종 (간단한 WASD 조작) |

## 의존성

- ROS 2 Jazzy
- px4_msgs (PX4 통신용 - 소스 빌드 필요)
- PX4 Autopilot (시뮬레이션/실제 비행용)

## 설치

```bash
# 1. 워크스페이스 생성
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. 패키지 복사/클론
cp -r robot_controller ~/ros2_ws/src/
# 또는
git clone <your-repo-url> robot_controller

# 3. px4_msgs 소스 빌드 (비행 기능 사용 시 - Jazzy는 소스 빌드 필요)
git clone https://github.com/PX4/px4_msgs.git
cd ~/ros2_ws
colcon build --packages-select px4_msgs
source install/setup.bash

# 4. robot_controller 빌드
cd ~/ros2_ws
colcon build --packages-select robot_controller

# 5. 소스
source install/setup.bash
```

## 실행 방법

### 1. RC카 키보드 조종 (지상 모드)

```bash
# 방법 1: launch 파일 사용
ros2 launch robot_controller teleop.launch.py

# 방법 2: 개별 노드 실행
# 터미널 1
ros2 run robot_controller wheel_controller_node

# 터미널 2
ros2 run robot_controller teleop_keyboard_node
```

### 2. 전체 시스템 실행 (지상+비행)

```bash
ros2 launch robot_controller bringup.launch.py
```

### 3. 시뮬레이션 (PX4 SITL)

```bash
# 터미널 1: PX4 SITL 실행
cd PX4-Autopilot
make px4_sitl gazebo

# 터미널 2: ROS 2 노드 실행
ros2 launch robot_controller simulation.launch.py

# 터미널 3: 키보드 조작
ros2 run robot_controller commander_node
```

## 토픽 구조

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 지상 모드 속도 명령 |
| `/flight/cmd_vel` | `geometry_msgs/Twist` | 비행 모드 속도 명령 |
| `/mode_command` | `std_msgs/String` | 모드 전환 명령 (TAKEOFF, LAND) |
| `/robot_mode` | `std_msgs/String` | 현재 모드 (GROUND, FLIGHT, TRANSITION) |
| `/wheel/motor_pwm` | `std_msgs/Int32MultiArray` | 바퀴 모터 PWM 출력 [left, right] |
| `/wheel/enable` | `std_msgs/String` | 바퀴 활성화/비활성화 (ENABLE/DISABLE) |
| `/wheel/status` | `std_msgs/String` | 바퀴 상태 |

## 키보드 조작

### teleop_keyboard_node (RC카 전용)

```
이동:
   q   w   e      (전진+좌회전, 전진, 전진+우회전)
   a   x   d      (좌회전, 정지, 우회전)
   z   s   c      (후진+좌회전, 후진, 후진+우회전)

속도 조절:
   r/f : 선속도 증가/감소
   t/g : 각속도 증가/감소

기타:
   스페이스바 : 긴급 정지
   Ctrl+C : 종료
```

### commander_node (통합 조작)

| 키 | 동작 |
|----|------|
| t | 이륙 (TAKEOFF) |
| l | 착륙 (LAND) |
| w/s | 전진/후진 |
| a/d | 좌회전/우회전 (지상) 또는 좌우이동 (비행) |
| q/e | 좌우 회전 (비행 모드) |
| r/f | 상승/하강 (비행 모드) |
| x | 정지 |

## 파라미터 설정

`config/params.yaml`에서 설정 가능:

```yaml
wheel_controller:
  ros__parameters:
    wheel_base: 0.3        # 바퀴 간 거리 (m)
    max_linear_vel: 1.0    # 최대 선속도 (m/s)
    max_angular_vel: 2.0   # 최대 각속도 (rad/s)

teleop_keyboard:
  ros__parameters:
    linear_speed: 0.5      # 기본 선속도 (m/s)
    angular_speed: 1.0     # 기본 각속도 (rad/s)
```

## 팀원

- 이강준, 김동욱, 이태호, 권현우, 권해찬, 송은서

## 라이선스

MIT License
