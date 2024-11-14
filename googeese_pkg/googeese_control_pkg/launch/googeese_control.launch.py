# googeese_control_pkg/launch/launch.py

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 이름
    package_name = 'googeese_control_pkg'

    # 노드 정의
    cmd_control_node = Node(
        package=package_name,
        executable='cmd_control_node',
        name='cmd_control_node',
        output='screen',
        # 필요한 경우 추가 인자나 환경 변수 설정 가능
        # parameters=[{'param_name': 'value'}],
    )

    command_node = Node(
        package=package_name,
        executable='command_node',
        name='command_node',
        output='screen',
    )

 
    # LaunchDescription에 모든 노드 추가
    return LaunchDescription([
        cmd_control_node,
        command_node,
    ])
