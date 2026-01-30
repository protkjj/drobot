from setuptools import setup
import os
from glob import glob

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Drobot Team',
    maintainer_email='drobot@example.com',
    description='Robot Controller ROS 2 Package - Ground + Flight Hybrid Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mode_manager_node = robot_controller.mode_manager_node:main',
            'wheel_controller_node = robot_controller.wheel_controller_node:main',
            'px4_interface_node = robot_controller.px4_interface_node:main',
            'commander_node = robot_controller.commander_node:main',
            'teleop_keyboard_node = robot_controller.teleop_keyboard_node:main',
        ],
    },
)
