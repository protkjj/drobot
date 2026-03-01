#!/bin/bash
set -e

# ROS2 환경 소싱
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash 2>/dev/null || true

exec "$@"
