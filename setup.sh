#!/bin/bash
set -e

DROBOT_DIR="$HOME/Desktop/drobot"

echo "============================="
echo "  DROBOT 개발환경 세팅 시작"
echo "============================="

# PX4-Autopilot 클론
if [ ! -d "$DROBOT_DIR/PX4-Autopilot" ]; then
    echo "[1/4] PX4-Autopilot 클론 중..."
    cd "$DROBOT_DIR"
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    cd PX4-Autopilot
    git checkout v1.16.1
    git submodule update --init --recursive
else
    echo "[1/4] PX4-Autopilot 이미 존재, 스킵"
fi

# PX4 의존성 설치
echo "[2/4] PX4 의존성 설치 중..."
bash "$DROBOT_DIR/PX4-Autopilot/Tools/setup/ubuntu.sh"

# Micro-XRCE-DDS-Agent 빌드
if [ ! -d "$DROBOT_DIR/Micro-XRCE-DDS-Agent" ]; then
    echo "[3/4] Micro-XRCE-DDS-Agent 빌드 중..."
    cd "$DROBOT_DIR"
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig
else
    echo "[3/4] Micro-XRCE-DDS-Agent 이미 존재, 스킵"
fi

# ROS2 워크스페이스 빌드
echo "[4/4] ros2_ws 빌드 중..."
cd "$DROBOT_DIR/ros2_ws"
colcon build

echo "============================="
echo "  세팅 완료!"
echo "============================="
echo ""
echo "사용법:"
echo "  터미널 1: cd ~/Desktop/drobot/PX4-Autopilot && make px4_sitl gz_x500"
echo "  터미널 2: MicroXRCEAgent udp4 -p 8888"
echo "  터미널 3: cd ~/Desktop/drobot/ros2_ws && source install/setup.bash && ros2 launch ..."
