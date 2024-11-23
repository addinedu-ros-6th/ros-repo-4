from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    yaml_file = os.path.join(
        get_package_share_directory('navigate_waypoints'),
        'navigate_waypoints',
        'points.yaml'
    )

    return LaunchDescription([
        Node(
            package='navigate_waypoints',
            executable='navigate_with_waypoints',  # 실행 파일 이름
            name='navigate_with_waypoints_node',
            output='screen',
            arguments=[yaml_file]  # 명령줄 인수로 yaml_path 전달
        )
    ])
