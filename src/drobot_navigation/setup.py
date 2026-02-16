import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drobot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # setup.py 내부 data_files
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # ✅ config 폴더 내 모든 yaml 파일을 설치하도록 설정
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='essong',
    description='Integrated Navigation and Simulation Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # scripts 폴더의 goal_navigator.py를 노드로 등록
            'goal_navigator = scripts.goal_navigator:main',
        ],
    },
)