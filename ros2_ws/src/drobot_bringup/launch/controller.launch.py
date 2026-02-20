#!/usr/bin/env python3
"""
Manual controller launch.

Starts the drobot_controller node and opens a separate terminal for keyboard teleop.
"""
import shutil
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_teleop_terminal_action(context):
    """Spawn teleop_twist_keyboard in a real terminal window."""
    open_terminal = context.launch_configurations.get('open_teleop_terminal', 'true')
    if open_terminal.lower() not in ('1', 'true', 'yes', 'on'):
        return []

    requested = context.launch_configurations.get('terminal_emulator', 'auto')
    if requested == 'auto':
        candidates = ['gnome-terminal', 'konsole', 'xfce4-terminal', 'mate-terminal', 'xterm']
    else:
        candidates = [requested]

    terminal = next((c for c in candidates if shutil.which(c)), None)
    if terminal is None:
        return [
            LogInfo(
                msg=(
                    '[controller.launch.py] No terminal emulator found. '
                    'Run teleop manually: ros2 run teleop_twist_keyboard teleop_twist_keyboard'
                )
            )
        ]

    teleop_cmd = 'ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash'
    if terminal == 'gnome-terminal':
        cmd = [terminal, '--', 'bash', '-lc', teleop_cmd]
    elif terminal == 'konsole':
        cmd = [terminal, '-e', 'bash', '-lc', teleop_cmd]
    elif terminal == 'xfce4-terminal':
        cmd = [terminal, '--command', f"bash -lc '{teleop_cmd}'"]
    elif terminal == 'mate-terminal':
        cmd = [terminal, '--', 'bash', '-lc', teleop_cmd]
    else:  # xterm and fallback terminals using -e
        cmd = [terminal, '-e', 'bash', '-lc', teleop_cmd]

    return [
        LogInfo(msg=f'[controller.launch.py] Opening teleop terminal with: {terminal}'),
        ExecuteProcess(cmd=cmd, output='screen'),
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    controller_node = Node(
        package='drobot_controller',
        executable='manual_controller',
        name='manual_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'open_teleop_terminal',
            default_value='true',
            description='Open a separate terminal window for teleop keyboard'
        ),
        DeclareLaunchArgument(
            'terminal_emulator',
            default_value='auto',
            description='Terminal emulator to use (auto, gnome-terminal, konsole, xterm, ...)'
        ),
        controller_node,
        OpaqueFunction(function=_build_teleop_terminal_action),
    ])
