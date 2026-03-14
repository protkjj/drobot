from setuptools import find_packages, setup

package_name = 'drobot_tracker'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jungwon',
    maintainer_email='kimjw0196@naver.com',
    description='Multi-object tracker with Kalman filter and Hungarian association',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'tracker_node = drobot_tracker.tracker_node:main',
        ],
    },
)
