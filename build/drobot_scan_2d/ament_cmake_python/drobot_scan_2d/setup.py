from setuptools import find_packages
from setuptools import setup

setup(
    name='drobot_scan_2d',
    version='2.0.0',
    packages=find_packages(
        include=('drobot_scan_2d', 'drobot_scan_2d.*')),
)
