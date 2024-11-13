import rclpy
import yaml
import math
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point

class NavigateTree(Node):
    def __init__(self, yaml_path):
        super().__init__('navigate_tree_node')

        # BasicNavigator 초기화
        self.navigator = BasicNavigator()
        self.subscription = self.create_subscription(Int32, 'goal_id', self.goal_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.plan_subscriber = self.create_subscription(Path, '/plan', self.plan_callback, 10)

        # YAML 파일에서 목표 및 경유지 로드
        self.load_points_from_yaml(yaml_path)
        self.planned_path_points = []  # 계획된 경로 점 리스트
        self.filtered_waypoints = []  # 필터링된 경유지 리스트
        self.initial_plan_received = False  # 초기 경로 수신 여부 플래그
        self.current_waypoint_index = 0  # 현재 이동 중인 경유지 인덱스
        self.previous_marker_ids = []  # 이전에 표시된 마커 ID 저장

    def load_points_from_yaml(self, yaml_path):
        # YAML 파일에서 목표 및 경유지, 경유지 설정 범위를 읽어오는 함수
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)
            self.goal_points = data['goal_points']
            self.waypoints = data['waypoints']
            self.range_threshold = data['range_threshold']
        self.get_logger().info('Loaded waypoints and goals from YAML')

    def goal_callback(self, msg):
        # goal_id를 수신하여 해당 목표를 찾고 설정
        goal_id = msg.data
        self.destination = next((goal['point'] for goal in self.goal_points if goal['id'] == goal_id), None)
        
        if self.destination:
            self.get_logger().info(f"Received goal ID {goal_id}. Destination set: {self.destination}")
            self.set_goal()
        else:
            self.get_logger().error(f"Goal ID {goal_id} not found in YAML.")

    def set_goal(self):
        # BasicNavigator에 목표 위치 설정
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = self.destination['x']
        goal_pose.pose.position.y = self.destination['y']
        goal_pose.pose.orientation.w = 1.0
        
        self.navigator.goToPose(goal_pose)
        self.get_logger().info("Goal set in BasicNavigator.")

    def plan_callback(self, msg):
        # 경로가 새로 갱신될 때마다 경유지를 필터링하고 시각화
        self.planned_path_points = [{'x': pose.pose.position.x, 'y': pose.pose.position.y} for pose in msg.poses]
        self.filter_waypoints_near_path()
        self.clear_previous_markers()  # 이전 마커 삭제
        self.visualize_path_range()
        self.current_waypoint_index = 0  # 경유지 인덱스 초기화
        self.navigate_to_next_waypoint()

    def clear_previous_markers(self):
        # 기존 마커들을 삭제하는 함수
        delete_marker_array = MarkerArray()
        for marker_id in self.previous_marker_ids:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = marker_id
            marker.action = Marker.DELETE
            delete_marker_array.markers.append(marker)
        self.marker_pub.publish(delete_marker_array)
        self.previous_marker_ids.clear()

    def filter_waypoints_near_path(self):
        # 계획된 경로를 기준으로 범위 내에 있는 경유지 필터링
        self.filtered_waypoints = []
        for waypoint in self.waypoints:
            is_on_path = any(
                self.calculate_distance(waypoint['point'], path_point) <= self.range_threshold
                for path_point in self.planned_path_points
            )
            if is_on_path:
                self.filtered_waypoints.append({'point': waypoint['point'], 'index': len(self.filtered_waypoints) + 1})

        self.get_logger().info(f"Filtered waypoints: {self.filtered_waypoints}")
        self.publish_markers(self.filtered_waypoints, self.destination)

    def navigate_to_next_waypoint(self):
        # 경유지를 순차적으로 탐색하며 부드럽게 이동
        if self.current_waypoint_index < len(self.filtered_waypoints):
            waypoint = self.filtered_waypoints[self.current_waypoint_index]
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = waypoint['point']['x']
            goal_pose.pose.position.y = waypoint['point']['y']
            goal_pose.pose.orientation.w = 1.0
            self.navigator.goToPose(goal_pose)

            # 목표 지점에 도착 시 콜백 등록
            self.navigator.taskResultCallback = self.on_waypoint_reached
        else:
            self.get_logger().info("All waypoints have been visited. Navigating to final destination...")
            self.set_goal()

    def on_waypoint_reached(self, result):
        # 경유지 도착 시 호출되는 함수
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index + 1}")
            self.current_waypoint_index += 1
            self.navigate_to_next_waypoint()
        else:
            self.get_logger().error("Failed to reach waypoint. Retrying the same waypoint...")
            self.navigate_to_next_waypoint()  # 실패 시 같은 경유지 재시도

    def visualize_path_range(self):
        # 경로를 따라 설정된 범위를 시각화하는 마커 생성
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = 999
        marker.type = Marker.LINE_STRIP
        marker.scale.x = self.range_threshold * 2
        marker.color.a = 0.3
        marker.color.b = 1.0

        for path_point in self.planned_path_points:
            p = Point()
            p.x = path_point['x']
            p.y = path_point['y']
            marker.points.append(p)

        self.marker_pub.publish(MarkerArray(markers=[marker]))
        self.previous_marker_ids.append(999)  # 생성된 마커 ID 저장

    def publish_markers(self, waypoints, destination):
        # 필터링된 경유지 및 목적지 마커를 표시
        marker_array = MarkerArray()
        for i, waypoint in enumerate(waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.pose.position.x = waypoint['point']['x']
            marker.pose.position.y = waypoint['point']['y']
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.g = 1.0
            marker_array.markers.append(marker)
            self.previous_marker_ids.append(i)  # 마커 ID 저장

            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.id = i + len(waypoints)
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.pose.position.x = waypoint['point']['x']
            text_marker.pose.position.y = waypoint['point']['y']
            text_marker.pose.position.z = 0.3
            text_marker.scale.z = 0.2
            text_marker.color.a = 1.0
            text_marker.text = str(i + 1)
            marker_array.markers.append(text_marker)
            self.previous_marker_ids.append(i + len(waypoints))

        destination_marker = Marker()
        destination_marker.header.frame_id = "map"
        destination_marker.id = len(waypoints) * 2
        destination_marker.type = Marker.SPHERE
        destination_marker.pose.position.x = destination['x']
        destination_marker.pose.position.y = destination['y']
        destination_marker.scale.x = destination_marker.scale.y = destination_marker.scale.z = 0.3
        destination_marker.color.a = 1.0
        destination_marker.color.r = 1.0
        marker_array.markers.append(destination_marker)
        self.previous_marker_ids.append(len(waypoints) * 2)  # 목적지 마커 ID 저장

        self.marker_pub.publish(marker_array)

    def calculate_distance(self, point1, point2):
        # 두 점 간의 유클리드 거리 계산
        return math.sqrt((point1['x'] - point2['x']) ** 2 + (point1['y'] - point2['y']) ** 2)

def main(args=None):
    rclpy.init(args=args)
    yaml_path = '/home/sehyung/minibot_ws/src/pinklab_minibot_robot/navigate_waypoints/navigate_waypoints/points.yaml'
    node = NavigateTree(yaml_path=yaml_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
