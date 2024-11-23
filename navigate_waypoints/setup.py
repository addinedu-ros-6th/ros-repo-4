from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'navigate_waypoints'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    include_package_data=True,  # 패키지 데이터 포함
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 모든 launch 파일을 설치
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # points.yaml 파일을 share/navigate_waypoints/navigate_waypoints에 복사
        ('share/' + package_name + '/navigate_waypoints', ['navigate_waypoints/points.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sehyung',
    maintainer_email='dkapqk8282@naver.com',
    description='Navigate Waypoints Node for Pinklab Minibot Robot',
    license='Apache License 2.0',
    # tests_require=['pytest'],  # 이 줄을 제거하여 경고 메시지 해결
    entry_points={
        'console_scripts': [
            'navigate_with_waypoints = navigate_waypoints.navigate_with_waypoints:main'  # 실행 파일 이름 일치
        ],
    },
)
