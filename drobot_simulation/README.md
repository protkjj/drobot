# Drobot Simulation Package

ROS 2 Jazzy + Gazebo Harmonic 시뮬레이션 패키지

## 폴더 구조

```
drobot_simulation/
├── launch/
│   ├── auto_explore.launch.py # 자동 탐색 (SLAM + Nav2 + Frontier)
│   ├── simulation.launch.py   # Gazebo + RViz 시뮬레이션
│   ├── display.launch.py      # RViz 시각화 (Gazebo 없이)
│   └── localization.launch.py # EKF 로컬라이제이션
├── urdf/                       # → drobot_description/urdf로 이동됨
├── meshes/                     # → drobot_description/meshes로 이동됨
│   └── drobot_body.stl        # 전체 로봇 STL (20MB)
├── worlds/                     # → drobot_description/worlds로 이동됨
├── config/
│   ├── nav2_params.yaml       # Nav2 설정 (VER0 자동 탐색용)
│   ├── slam_params.yaml       # SLAM 설정
│   ├── ekf.yaml               # EKF 설정
│   └── ros_gz_bridge.yaml     # Gazebo 브릿지 설정
├── CMakeLists.txt
└── package.xml
```

## 의존성 설치

```bash
# ROS 2 Jazzy 기본 패키지
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-robot-state-publisher
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-rviz2
sudo apt install ros-jazzy-teleop-twist-keyboard

# SLAM 및 Navigation 패키지
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-nav2-bringup
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

**조작 키:**
```
이동:
   u    i    o
   j    k    l
   m    ,    .

i/,  : 전진/후진
j/l  : 좌회전/우회전
u/o  : 전진 + 좌/우회전
m/.  : 후진 + 좌/우회전
k    : 정지

속도 조절:
q/z  : 선속도 증가/감소 (10%)
w/x  : 각속도 증가/감소 (10%)
```

### 4. 토픽으로 직접 제어
```bash
# 전진 (0.5 m/s)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# 회전 (0.5 rad/s)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# 정지
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
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
| `/camera/image_raw` | `sensor_msgs/Image` | 카메라 영상 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 카메라 정보 |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM 지도 (SLAM 실행 시) |

## 파라미터 수정

`urdf/drobot.urdf.xacro` 상단의 property 값 수정:

```xml
<!-- 로봇 크기 (미터) -->
<xacro:property name="body_length" value="0.30"/>
<xacro:property name="body_width" value="0.20"/>
<xacro:property name="wheel_radius" value="0.05"/>
```

---

## SLAM 환경 구축 및 실습

SLAM(Simultaneous Localization and Mapping)을 사용하여 로봇이 환경을 탐색하면서 실시간으로 지도를 생성합니다.

### Step 1. 필수 패키지 설치

```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-teleop-twist-keyboard
```

### Step 2. 시뮬레이션 실행

```bash
# 터미널 1: 시뮬레이션 실행
ros2 launch drobot_simulation simulation.launch.py
```

**데이터 확인** (새 터미널에서):
```bash
# 라이다 토픽 확인
ros2 topic list    # /scan이 있어야 함
ros2 topic echo /scan --once   # 숫자가 나오면 성공

# TF 확인
ros2 run tf2_tools view_frames
# 생성된 frames.pdf에서 끊어진 곳이 없는지 확인
```

### Step 3. SLAM Toolbox 실행

```bash
# 터미널 2: SLAM 실행
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

> `online_async`는 실시간으로 비동기식(끊김 없이)으로 지도를 그리는 모드입니다.

### Step 4. RViz2에서 지도 보기

시뮬레이션 런치 파일에 RViz가 포함되어 있지만, 수동으로 실행하려면:

```bash
ros2 run rviz2 rviz2
```

**RViz 설정:**
1. 왼쪽 하단 **[Add]** 버튼 클릭
2. **By topic** 탭 → `/map` → **Map** 선택 후 OK
3. **Global Options** → **Fixed Frame**을 `map`으로 변경
4. 화면에 회색/검은색/흰색 지도가 보이면 성공

### Step 5. 로봇 움직이며 지도 그리기

```bash
# 터미널 3: 키보드 조종
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**조작법:**
- `i` : 전진
- `,` : 후진
- `j` : 좌회전
- `l` : 우회전
- `k` : 정지
- `q/z` : 속도 증가/감소

로봇을 조종해서 맵 전체를 돌면 RViz에서 지도가 점점 넓어집니다.

### Step 6. 지도 파일로 저장하기

**SLAM 노드를 끄지 않은 상태에서** 새 터미널을 열고:

```bash
ros2 run nav2_map_server map_saver_cli -f my_first_map
```

**결과 파일:**
- `my_first_map.pgm` : 지도 이미지 파일
- `my_first_map.yaml` : 지도 설정 파일 (해상도, 원점 좌표 등)

### 문제 해결

**지도가 안 그려질 때:**
1. 라이다 토픽 이름 확인: `/scan`이 아닌 경우 파라미터 수정 필요
   ```bash
   ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true scan_topic:=/lidar_scan
   ```

2. TF 체인 확인: `odom` → `base_footprint` → `base_link` → `lidar_link` 연결 필요

**RViz에서 Fixed Frame 오류:**
- Global Options → Fixed Frame을 `map`으로 설정

---

## 센서

### LiDAR
- 토픽: `/scan`
- 360도 스캔, 12m 범위
- RViz에서 빨간 점으로 표시

### 카메라
- 토픽: `/camera/image_raw`, `/camera/camera_info`
- 640x480 해상도, 30fps
- RViz에서 Image 디스플레이로 확인

### IMU
- 토픽: `/imu`
- 100Hz 업데이트
