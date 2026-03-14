#!/usr/bin/env python3
"""
ros2 launch drobot_bringup total_ui.launch.py
"""
import sys
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[sys.executable, "-m", "drobot_bringup.total_ui"],
            output="screen",
            sigterm_timeout="0",
            sigkill_timeout="0",
            emulate_tty=True,
        ),
    ])
