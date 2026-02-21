# Patch Note

## 범위
- 작업 위치: `~/Desktop/drobot/ros2_ws`
- 대상 패키지: `drobot_description`, `drobot_bringup`

## 1) 신규 모델 파일/폴더 추가
- `drobot_description`에 하위 폴더 추가:
  - `urdf_sub/`
  - `mesh_sub/`
- `~/Downloads`에서 복사:
  - URDF: `drobotv0.urdf` (및 `linkage_practice_added.urdf`)
  - Mesh: STL 11개

## 2) `drobotv0` 경로 정리
- 파일: `src/drobot_description/urdf_sub/drobotv0.urdf`
- 변경:
  - mesh 경로를 `package://drobot_description/mesh_sub/...`로 통일
  - 파일명 불일치 없음 확인

## 3) xacro 기반 전환
- 파일 추가: `src/drobot_description/urdf_sub/drobotv0.urdf.xacro`
- 변경:
  - xacro namespace 추가 (`xmlns:xacro`)
- 런치 전환:
  - 파일: `src/drobot_description/launch/new_display.launch.py`
  - `drobotv0.urdf.xacro`를 `xacro`로 처리하여 RViz 표시

## 4) 조인트 회전 방향/미믹 수정
- 파일: `src/drobot_description/urdf_sub/drobotv0.urdf.xacro`
- 회전축 반전:
  - `armjoint_left`
  - `servo_RF`
  - `wheeljoint_RF`
  - `wheeljoint_RR`
- mimic 보정:
  - `servo_LR` -> `servo_LF` `multiplier=-1`
  - `servo_RR` -> `servo_RF` `multiplier=-1`
- 추가 mimic:
  - `servo_RF` -> `servo_LF` `multiplier=1`
  - `armjoint_right` -> `armjoint_left` `multiplier=1`

## 5) Gazebo 신규 런치 추가
- 파일 추가: `src/drobot_description/launch/new_gazebo.launch.py`
- 목적:
  - 신규 모델 `urdf_sub/drobotv0.urdf.xacro` 스폰
- `CMakeLists.txt` 보완:
  - `urdf_sub`, `mesh_sub` 설치 대상으로 추가

## 6) Gazebo 로딩 오류 대응 (질량 0 이슈)
- 파일: `src/drobot_description/urdf_sub/drobotv0.urdf.xacro`
- 변경:
  - `L_shaped_LF`, `L_shaped_LR`의 mass/inertia 0 값을 양수로 수정
- 효과:
  - urdf2sdf 변환 시 링크/조인트 누락 완화

## 7) `cmd_vel` 구동 플러그인 추가
- 파일: `src/drobot_description/urdf_sub/drobotv0.urdf.xacro`
- 추가 Gazebo systems:
  - JointStatePublisher
  - DiffDrive (`/cmd_vel`)
  - OdometryPublisher (`/odom`)

## 8) RViz 전용 주행 테스트(비-Gazebo) 기능 추가
### 8-1. fake odom + tf + wheel joint state
- 파일: `src/drobot_bringup/drobot_bringup/fake_diff_odom.py`
- 기능:
  - `/cmd_vel` 구독
  - `/odom` 발행
  - `odom -> base_link` TF 발행
  - `/joint_states` 발행 (바퀴 회전 시각화)

### 8-2. RViz 테스트 런치
- 파일: `src/drobot_bringup/launch/rviz_teleop_test.launch.py`
- 구성:
  - `robot_state_publisher`
  - `rviz2`
  - `fake_diff_odom`

### 8-3. 조인트 키보드 텔레옵 추가
- 파일 추가: `src/drobot_bringup/drobot_bringup/joint_key_teleop.py`
- 키 매핑:
  - `r/f`: servo up/down
  - `t/g`: arm up/down
  - `space`: joint reset
  - `q`: quit
- 토픽:
  - publish `/joint_key_cmd` (`std_msgs/String`)
- `fake_diff_odom.py`에 `/joint_key_cmd` 처리 추가

## 9) 패키지 설정 반영
- 파일: `src/drobot_bringup/setup.py`
  - console_scripts 추가:
    - `fake_diff_odom`
    - `joint_key_teleop`
- 파일: `src/drobot_bringup/package.xml`
  - 의존성 추가:
    - `geometry_msgs`
    - `nav_msgs`
    - `sensor_msgs`
    - `std_msgs`
    - `tf2_ros`

## 10) 실행 명령 정리
### 공통
```bash
cd ~/Desktop/drobot/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 신규 모델 RViz
```bash
ros2 launch drobot_description new_display.launch.py
```

### 신규 모델 Gazebo
```bash
ros2 launch drobot_description new_gazebo.launch.py
```

### RViz 주행 테스트(비-Gazebo)
```bash
# 터미널 1
ros2 launch drobot_bringup rviz_teleop_test.launch.py

# 터미널 2 (이동)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 터미널 3 (servo/arm 각도)
ros2 run drobot_bringup joint_key_teleop
```

## 참고
- 새 터미널마다 아래 재실행 필요:
```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/drobot/ros2_ws/install/setup.bash
```

## 11) 2026-02-21 Gazebo 조인트 안정화 + 개별 조인트 제어 추가
- 목표:
  - `new_gazebo.launch.py` 실행 시 모델 조인트가 무너져 보이는 문제 해결
  - Gazebo에서 각 조인트를 토픽으로 직접 제어 가능하도록 구성

- 파일: `src/drobot_description/urdf_sub/drobotv0.urdf.xacro`
  - 수정:
    - LF 바퀴 조인트 오타 수정: `wheejoint_LF` -> `wheeljoint_LF`
    - `servo_*`, `armjoint_*`를 `revolute`로 복원
    - 조인트 안정화용 `limit`, `dynamics(damping/friction)` 추가
    - `gz::sim::systems::JointPositionController` 플러그인 6개 추가
      - `/servo_LF/cmd_pos`
      - `/servo_LR/cmd_pos`
      - `/servo_RF/cmd_pos`
      - `/servo_RR/cmd_pos`
      - `/armjoint_left/cmd_pos`
      - `/armjoint_right/cmd_pos`

- 파일: `src/drobot_description/urdf_sub/drobotv0.urdf`
  - 수정:
    - LF 바퀴 조인트 오타 정합: `wheejoint_LF` -> `wheeljoint_LF`

- 파일: `src/drobot_bringup/drobot_bringup/fake_diff_odom.py`
  - 수정:
    - 조인트 이름 정합: `wheejoint_LF` -> `wheeljoint_LF`

- 검증:
  - `xacro` 전개 확인 완료
  - `colcon build --packages-select drobot_description drobot_bringup` 성공
  - `colcon build --packages-select drobot_description` 재검증 성공
