import rclpy
import yaml
import math
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from rclpy.qos import QoSProfile

class NavigateTree(Node):
    def __init__(self, yaml_path):
        super().__init__('navigate_tree_node')
        
        # BasicNavigator 초기화
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # QoS 설정
        qos = QoSProfile(depth=10)

        # 구독 및 퍼블리셔 설정
        self.subscription = self.create_subscription(Int32, 'goal_id', self.goal_callback, qos)
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', qos)
        self.plan_subscriber = self.create_subscription(Path, '/plan', self.plan_callback, qos)

        # YAML 파일에서 목표 및 경유지 로드
        self.load_points_from_yaml(yaml_path)
        self.all_waypoints = self.waypoints.copy()
        self.filtered_waypoints = []
        self.current_waypoint_index = 0

        # 마커 ID 관리
        self.filtered_marker_ids = []
        self.path_range_marker_id = 999

        # 경로가 이미 처리되었는지 여부를 추적하는 플래그
        self.plan_processed = False

        # 모든 경유지를 시각화
        self.visualize_all_waypoints()

    def load_points_from_yaml(self, yaml_path):
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)
            self.goal_points = data['goal_points']
            self.waypoints = data['waypoints']
            self.range_threshold = data['range_threshold']
        self.get_logger().info('YAML에서 경유지와 목표를 로드했습니다.')

    def visualize_all_waypoints(self):
        marker_array = MarkerArray()
        for i, waypoint in enumerate(self.all_waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = 1000 + i
            marker.type = Marker.SPHERE
            marker.pose.position.x = waypoint['point']['x']
            marker.pose.position.y = waypoint['point']['y']
            marker.pose.position.z = waypoint.get('z', 0.0)
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.color.a = 0.5
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        self.get_logger().info("모든 경유지를 회색으로 시각화했습니다.")

    def goal_callback(self, msg):
        goal_id = msg.data
        self.destination = next((goal['point'] for goal in self.goal_points if goal['id'] == goal_id), None)

        if self.destination:
            self.get_logger().info(f"수신된 목표 ID {goal_id}. 목적지 설정: {self.destination}")
            # 목표 위치 설정
            self.set_goal()
            self.get_logger().info("목표가 설정되었으며, goToPose가 호출되었습니다.")
        else:
            self.get_logger().error(f"YAML에서 목표 ID {goal_id}를 찾을 수 없습니다.")

    def clear_filtered_markers(self):
        marker_array = MarkerArray()

        for marker_id in self.filtered_marker_ids:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = marker_id
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = self.path_range_marker_id
        marker.action = Marker.DELETE
        marker_array.markers.append(marker)

        if marker_array.markers:
            self.marker_pub.publish(marker_array)
            self.get_logger().info("필터링된 경유지 및 경로 범위 마커를 삭제했습니다.")

        self.filtered_marker_ids.clear()

    def set_goal(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = self.destination['x']
        goal_pose.pose.position.y = self.destination['y']
        goal_pose.pose.orientation.w = 1.0

        self.navigator.goToPose(goal_pose)
        if not self.navigator.waitUntilNav2Active():
            self.get_logger().error("Nav2가 활성화되지 않았습니다.")
            return

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("최종 목적지에 성공적으로 도착했습니다.")
            self.clear_all_markers()
        else:
            self.get_logger().error("최종 목적지로 이동하는 데 실패했습니다.")

    def plan_callback(self, msg):
        # 경유지가 설정된 경우 다시 경로를 처리하지 않도록
        if self.plan_processed:
            self.get_logger().info("경유지가 이미 설정되어 추가 경로 처리를 건너뜁니다.")
            return

        self.get_logger().info("plan 토픽 수신, 경로 계산을 시작합니다.")
        self.planned_path_points = [{'x': pose.pose.position.x, 'y': pose.pose.position.y} for pose in msg.poses]
        self.filter_waypoints_near_path()
        self.get_logger().info("경로가 필터링되었으며 이동을 시작합니다.")
        self.set_full_path()

    def filter_waypoints_near_path(self):
        self.filtered_waypoints = []
        for waypoint in self.waypoints:
            is_on_path = any(
                self.calculate_distance(waypoint['point'], path_point) <= self.range_threshold
                for path_point in self.planned_path_points
            )
            if is_on_path:
                self.filtered_waypoints.append({'point': waypoint['point'], 'index': len(self.filtered_waypoints) + 1})

        self.get_logger().info(f"필터링된 경유지 수: {len(self.filtered_waypoints)}")
        self.publish_filtered_waypoints_markers()

    def publish_filtered_waypoints_markers(self):
        marker_array = MarkerArray()
        for i, waypoint in enumerate(self.filtered_waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = 2000 + i
            marker.type = Marker.SPHERE
            marker.pose.position.x = waypoint['point']['x']
            marker.pose.position.y = waypoint['point']['y']
            marker.pose.position.z = waypoint.get('z', 0.0)
            marker.scale.x = marker.scale.y = marker.scale.z = 0.25
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker_array.markers.append(marker)
            self.filtered_marker_ids.append(2000 + i)

            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.id = 3000 + i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.pose.position.x = waypoint['point']['x']
            text_marker.pose.position.y = waypoint['point']['y']
            text_marker.pose.position.z = waypoint.get('z', 0.0) + 0.3
            text_marker.scale.z = 0.3
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = str(waypoint['index'])
            marker_array.markers.append(text_marker)
            self.filtered_marker_ids.append(3000 + i)

        self.marker_pub.publish(marker_array)
        self.get_logger().info("필터링된 경유지를 파란색으로 시각화하고 이동 순서를 표시했습니다.")

    def set_full_path(self):
        path = Path()
        path.header.frame_id = "map"
        for waypoint in self.filtered_waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = waypoint['point']['x']
            pose.pose.position.y = waypoint['point']['y']
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.navigator.followPath(path)
        self.plan_processed = True  # 경로 설정을 한 번만 처리하도록 플래그 설정
        self.get_logger().info("경로가 설정되었습니다.")

    def calculate_distance(self, point1, point2):
        return math.sqrt((point1['x'] - point2['x']) ** 2 + (point1['y'] - point2['y']) ** 2)

def main(args=None):
    rclpy.init(args=args)
    yaml_path = '/home/sehyung/minibot_ws/src/pinklab_minibot_robot/navigate_waypoints/navigate_waypoints/points.yaml'
    node = NavigateTree(yaml_path=yaml_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('키보드 인터럽트로 종료 요청됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
