from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    DeclareLaunchArgument('camera_test', default_value='false'),  # 기본값을 'false'로 명시
        Node(
            package='aruco_follower',
            executable='aruco_follower',
            name='aruco_follower',
            output='screen',
            parameters=[{'camera_test': LaunchConfiguration('camera_test')}]
        )
    ])
