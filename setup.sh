#!/bin/bash
set -e

DROBOT_DIR="$HOME/Desktop/drobot"

# venv 활성화되어 있으면 해제 (ROS2 빌드 충돌 방지)
if [ -n "$VIRTUAL_ENV" ]; then
    echo "Python venv 감지 ($VIRTUAL_ENV) -> 비활성화"
    deactivate 2>/dev/null || true
    export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "$VIRTUAL_ENV" | tr '\n' ':' | sed 's/:$//')
    unset VIRTUAL_ENV
fi

# ROS2 환경 로드
source /opt/ros/jazzy/setup.bash

echo "============================="
echo "  DROBOT 개발환경 세팅 시작"
echo "============================="

# 1. PX4-Autopilot (protkjj fork, drobot/vtol-rover-mode 브랜치)
if [ ! -d "$DROBOT_DIR/PX4-Autopilot" ]; then
    echo "[1/5] PX4-Autopilot 클론 중 (drobot/vtol-rover-mode 브랜치)..."
    cd "$DROBOT_DIR"
    git clone -b drobot/vtol-rover-mode https://github.com/protkjj/PX4-Autopilot.git
    cd PX4-Autopilot
    git submodule update --init --recursive
else
    echo "[1/5] PX4-Autopilot 이미 존재 -> pull 업데이트"
    cd "$DROBOT_DIR/PX4-Autopilot"
    git pull --rebase origin drobot/vtol-rover-mode
    git submodule update --init --recursive
fi

# 2. PX4 의존성 설치
echo "[2/5] PX4 의존성 설치 중..."
bash "$DROBOT_DIR/PX4-Autopilot/Tools/setup/ubuntu.sh"

# 3. Micro-XRCE-DDS-Agent
if [ ! -d "$DROBOT_DIR/Micro-XRCE-DDS-Agent" ]; then
    echo "[3/5] Micro-XRCE-DDS-Agent 클론 및 빌드 중..."
    cd "$DROBOT_DIR"
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir -p build && cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig
else
    echo "[3/5] Micro-XRCE-DDS-Agent 이미 존재, 스킵"
fi

# 4. px4_msgs (PX4 uORB 메시지 ROS2 정의)
if [ ! -d "$DROBOT_DIR/ros2_ws/src/px4_msgs" ]; then
    echo "[4/5] px4_msgs 클론 중..."
    cd "$DROBOT_DIR/ros2_ws/src"
    git clone https://github.com/PX4/px4_msgs.git
else
    echo "[4/5] px4_msgs 이미 존재, 스킵"
fi

# 5. ROS2 워크스페이스 빌드 (캐시 초기화 후 클린 빌드)
echo "[5/5] ros2_ws 빌드 중..."
cd "$DROBOT_DIR/ros2_ws"
rm -rf build install log
colcon build

echo "============================="
echo "  세팅 완료!"
echo "============================="
echo ""
echo "최종 폴더 구조:"
echo "  drobot/"
echo "    PX4-Autopilot/          (protkjj/drobot/vtol-rover-mode)"
echo "    Micro-XRCE-DDS-Agent/   (DDS 브릿지)"
echo "    ros2_ws/                 (ROS2 워크스페이스)"
echo "      src/px4_msgs/         (PX4 uORB 메시지 정의)"
echo ""
echo "사용법:"
echo "  ros2 launch drobot_bringup total_ui.launch.py"
