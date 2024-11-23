import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray

class NavigateTree(Node):
    def __init__(self, yaml_path):
        super().__init__('navigate_bt_node')
        self.load_points_from_yaml(yaml_path)  # YAML 파일에서 경유지와 목적지 로드

        # RViz 마커 표시를 위한 'visualization_marker_array' 퍼블리셔 설정
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        # 모든 목적지 및 경유지 마커를 퍼블리시
        self.publish_all_markers()

    # YAML 파일에서 경유지 및 목적지 로드
    def load_points_from_yaml(self, yaml_path):
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)
            self.goal_points = data['goal_points']  # 목표 지점 리스트
            self.waypoints = data['waypoints']  # 경유지 리스트
            self.range_threshold = data['range_threshold']  # 경유지 범위
        self.get_logger().info(f'Loaded waypoints and goals')

    # 모든 목적지 및 경유지 마커를 한 번에 생성하고 퍼블리시
    def publish_all_markers(self):
        marker_array = MarkerArray()

        # 모든 경유지 마커 생성
        for i, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint['point']['x']
            marker.pose.position.y = waypoint['point']['y']
            marker.pose.position.z = 0.1
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

            # 경유지 번호 텍스트 마커 추가
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.id = i + len(self.waypoints)
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = waypoint['point']['x']
            text_marker.pose.position.y = waypoint['point']['y']
            text_marker.pose.position.z = 0.3  # 텍스트 마커를 약간 높여서 표시
            text_marker.scale.z = 0.15  # 폰트 크기 축소
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = f"W{i+1}"  # 경유지 번호
            marker_array.markers.append(text_marker)

        # 모든 목적지 마커 생성
        for j, goal in enumerate(self.goal_points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = j + len(self.waypoints) * 2
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = goal['point']['x']
            marker.pose.position.y = goal['point']['y']
            marker.pose.position.z = 0.1
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

            # 목적지 번호 텍스트 마커 추가
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.id = j + len(self.waypoints) * 3
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = goal['point']['x']
            text_marker.pose.position.y = goal['point']['y']
            text_marker.pose.position.z = 0.3  # 텍스트 마커를 약간 높여서 표시
            text_marker.scale.z = 0.15  # 폰트 크기 축소
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = f"G{j+1}"  # 목적지 번호
            marker_array.markers.append(text_marker)

        # MarkerArray 퍼블리시
        self.marker_pub.publish(marker_array)
        self.get_logger().info("Published all markers for waypoints and goals")


def main(args=None):
    rclpy.init(args=args)

    # YAML 파일 경로 직접 지정
    yaml_path = '/home/sehyung/minibot_ws/src/navigate_waypoints/navigate_waypoints/points.yaml'  # YAML 파일 경로 직접 지정
    node = NavigateTree(yaml_path=yaml_path)  # NavigateTree 노드 생성
    rclpy.spin(node)  # 노드 실행
    node.destroy_node()  # 노드 종료 시 삭제
    rclpy.shutdown()  # ROS 2 종료

if __name__ == '__main__':
    main()
