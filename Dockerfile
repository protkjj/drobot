FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ============================================
# 1. 시스템 의존성
# ============================================
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    python3-pip \
    python3-tk \
    wget \
    curl \
    terminator \
    # ROS2 Nav2 / SLAM / Gazebo
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf2-ros \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# ============================================
# 2. Python 의존성 (YOLO, OpenCV)
# ============================================
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
    ultralytics \
    opencv-python-headless

# ============================================
# 3. Micro-XRCE-DDS-Agent 빌드 (PX4 <-> ROS2 브릿지)
# ============================================
WORKDIR /root
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    && cd Micro-XRCE-DDS-Agent \
    && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && cd /root && rm -rf Micro-XRCE-DDS-Agent

# ============================================
# 4. ROS2 워크스페이스 복사 및 빌드
# ============================================
WORKDIR /root/ros2_ws
COPY ros2_ws/src ./src

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build \
    && echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# ============================================
# 5. 엔트리포인트
# ============================================
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
