from setuptools import find_packages, setup
import os
from glob import glob # 파일 경로를 묶어서 처리하기 위해 필요합니다

package_name = 'drobot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. 런치 파일 등록 (launch 폴더 안의 모든 .py 파일을 share 폴더로 복사)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. 설정 파일 등록 (config 폴더 안의 모든 .yaml 파일을 share 폴더로 복사)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='essong',
    maintainer_email='essong@todo.todo',
    description='Drobot Navigation Package',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # 3. 노드 실행 명령어 등록
            # 형식: '명령어이름 = 패키지이름.파일명:메인함수이름'
            'goal_navigator = drobot_navigation.goal_navigator:main',
        ],
    },
)