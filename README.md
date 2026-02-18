# DROBOT

드론-로버 하이브리드 변형 로봇

## 초기 세팅 (최초 1회) 중요 !!

```bash
cd ~/Desktop
git clone https://github.com/protkjj/drobot.git
cd drobot
bash setup.sh
```

비밀번호 한 번 입력 후 자동으로 PX4, DDS Agent, ROS2 빌드까지 완료됩니다.

## 실행 방법

터미널 3개를 열고 각각 실행:

```bash
# 터미널 1: PX4 SITL + Gazebo
cd ~/Desktop/drobot/PX4-Autopilot && make px4_sitl gz_x500

# 터미널 2: DDS 브릿지
MicroXRCEAgent udp4 -p 8888

# 터미널 3: ROS2 노드
cd ~/Desktop/drobot/ros2_ws && source install/setup.bash
ros2 launch drobot_bringup drobot_sim.launch.py
```

## 코드 업데이트 받기

```bash
cd ~/Desktop/drobot
git pull
cd ros2_ws && colcon build
```

## 폴더 구조

```
drobot/
├── setup.sh                    # 환경 세팅 스크립트
├── README.md
├── ros2_ws/                    # ROS2 워크스페이스 (우리 코드)
│   └── src/
│       ├── drobot_msgs/
│       ├── drobot_description/
│       ├── drobot_bringup/
│       ├── drobot_firmware_bridge/
│       ├── drobot_perception/
│       ├── drobot_navigation/
│       ├── drobot_strategy/
│       ├── drobot_control/
│       ├── drobot_gazebo/
│       └── px4_msgs/
├── PX4-Autopilot/              # setup.sh이 자동 클론
└── Micro-XRCE-DDS-Agent/       # setup.sh이 자동 클론+빌드
```
