from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navigate_waypoints'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'navigate_waypoints.navigate_with_bt'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigate_launch.py']),  # launch 파일 경로 설정
        ('share/' + package_name + '/navigate_waypoints', ['navigate_waypoints/points.yaml']),  # YAML 파일 경로 설정
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sehyung',
    maintainer_email='dkapqk8282@naver.com',
    description='A ROS 2 package to navigate through waypoints using a behavior tree.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate_with_bt = navigate_waypoints.navigate_with_bt:main'
        ],
    },
)