# Drobot Simulation Package

ROS 2 Jazzy + Gazebo Harmonic 시뮬레이션 패키지

## 폴더 구조

```
drobot_simulation/
├── launch/
│   ├── simulation.launch.py   # Gazebo 시뮬레이션
│   └── display.launch.py      # RViz 시각화 (Gazebo 없이)
├── urdf/
│   └── drobot.urdf.xacro      # 로봇 모델
├── meshes/                     # STL 파일 (여기에 넣으세요)
│   ├── drone_body.stl         # 본체 (Onshape에서 export)
│   └── propeller.stl          # 프로펠러
├── worlds/
│   └── empty.sdf              # Gazebo 월드
├── config/
│   └── display.rviz           # RViz 설정
├── CMakeLists.txt
└── package.xml
```

## 의존성 설치

```bash
# ROS 2 Jazzy
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-robot-state-publisher
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-rviz2
```

## 빌드

```bash
cd ~/ros2_ws/src
# drobot_simulation 폴더를 여기에 복사

cd ~/ros2_ws
colcon build --packages-select drobot_simulation
source install/setup.bash
```

## 실행

### 1. RViz로 URDF 확인 (Gazebo 없이)
```bash
ros2 launch drobot_simulation display.launch.py
```

### 2. Gazebo 시뮬레이션
```bash
ros2 launch drobot_simulation simulation.launch.py
```

### 3. 키보드로 로봇 조종
```bash
# 새 터미널에서
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## STL 파일 추가하기

1. Onshape에서 STL export
2. `meshes/` 폴더에 파일 넣기
3. `urdf/drobot.urdf.xacro` 파일에서 주석 해제:

```xml
<!-- 이 줄을 찾아서 -->
<box size="${body_length} ${body_width} ${body_height}"/>

<!-- 이렇게 변경 -->
<mesh filename="package://drobot_simulation/meshes/drone_body.stl" scale="0.001 0.001 0.001"/>
```

## 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 속도 명령 |
| `/odom` | `nav_msgs/Odometry` | 오도메트리 |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR 데이터 |
| `/imu` | `sensor_msgs/Imu` | IMU 데이터 |
| `/joint_states` | `sensor_msgs/JointState` | 조인트 상태 |

## 파라미터 수정

`urdf/drobot.urdf.xacro` 상단의 property 값 수정:

```xml
<!-- 로봇 크기 (미터) -->
<xacro:property name="body_length" value="0.30"/>
<xacro:property name="body_width" value="0.20"/>
<xacro:property name="wheel_radius" value="0.05"/>
```

## 팀원

이강준, 김동욱, 이태호, 권현우, 권해찬, 송은서
