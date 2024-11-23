import rclpy
import yaml
import math
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker, MarkerArray
import py_trees
import sys

class NavigateThroughPosesAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, action_client, waypoints, destination):
        super().__init__(name)
        self.action_client = action_client
        self.waypoints = waypoints  # 경유지 리스트
        self.destination = destination  # 최종 목적지 좌표

    def initialise(self):
        goal_msg = NavigateThroughPoses.Goal()
        
        # 모든 경유지 좌표를 goal_msg에 추가
        for point in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = point['x']
            pose.pose.position.y = point['y']
            pose.pose.orientation.w = 1.0
            goal_msg.poses.append(pose)
        
        # 최종 목적지 좌표를 goal_msg에 추가
        destination_pose = PoseStamped()
        destination_pose.header.frame_id = "map"
        destination_pose.pose.position.x = self.destination['x']
        destination_pose.pose.position.y = self.destination['y']
        destination_pose.pose.orientation.w = 1.0
        goal_msg.poses.append(destination_pose)

        # Action 서버가 준비될 때까지 기다린 후, 목표 전송
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)

    def update(self):
        if self._send_goal_future.done():
            return py_trees.common.Status.SUCCESS  # 완료되면 성공 상태 반환
        else:
            return py_trees.common.Status.RUNNING  # 아직 진행 중이면 실행 중 상태 반환

class NavigateTree(Node):
    def __init__(self, yaml_path):
        super().__init__('navigate_bt_node')
        self.action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')  # 경로 이동 action client 생성
        self.tree = None
        self.load_points_from_yaml(yaml_path)  # YAML 파일에서 경유지와 목적지 로드
        
        # 목표 ID 수신을 위한 'goal_id' 토픽 구독 설정
        self.subscription = self.create_subscription(
            Int32, 'goal_id', self.goal_callback, 10
        )
        
        # RViz 마커 표시를 위한 'visualization_marker_array' 퍼블리셔 설정
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

    def load_points_from_yaml(self, yaml_path):
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)
            self.goal_points = data['goal_points']  # 목표 지점 리스트
            self.waypoints = data['waypoints']  # 경유지 리스트
            self.range_threshold = data['range_threshold']  # 경유지 범위
        self.get_logger().info(f'Loaded waypoints and goals')

    def calculate_distance(self, point1, point2):
        return math.sqrt((point1['x'] - point2['x'])**2 + (point1['y'] - point2['y'])**2)

    # 특정 목적지 기준으로 범위 내에 있는 경유지 필터링
    # 최종 목적지가 경유지보다 가까운 경우 해당 경유지는 제외
    def filter_waypoints_within_range(self, destination):
        selected_waypoints = []
        for waypoint in self.waypoints:
            distance_to_destination = self.calculate_distance(waypoint['point'], destination['point'])
            distance_to_robot = self.calculate_distance({'x': 0, 'y': 0}, waypoint['point'])  # 로봇 위치 기준 예시
            if distance_to_destination <= self.range_threshold and distance_to_destination < distance_to_robot:
                selected_waypoints.append(waypoint['point'])
        return selected_waypoints

    # 목표 ID 수신 시 호출되는 콜백 함수
    def goal_callback(self, msg):
        goal_id = msg.data  # 수신한 목표 ID
        self.get_logger().info(f"Received goal ID: {goal_id}")
        
        # 목표 ID에 해당하는 목적지 찾기
        destination = next((goal['point'] for goal in self.goal_points if goal['id'] == goal_id), None)
        
        if destination is not None:
            self.get_logger().info(f"Found destination for ID {goal_id}: {destination}")
            waypoints_in_range = self.filter_waypoints_within_range({'point': destination})
            self.get_logger().info(f"Waypoints in range for ID {goal_id}: {waypoints_in_range}")
            
            # RViz에 경유지 및 목적지 마커 퍼블리시
            self.get_logger().info("About to call publish_markers")  # 추가 디버그 로그
            self.publish_markers(waypoints_in_range, destination)
            self.get_logger().info("Called publish_markers successfully")  # 호출 후 로그
            
            # 경로 행동 트리 생성 및 실행
            self.get_logger().info("Creating behavior tree")
            self.create_behavior_tree(waypoints_in_range, destination)
        else:
            self.get_logger().error(f"Goal ID {goal_id} not found in goal_points.")

    # 경유지 및 목적지를 포함한 행동 트리 생성
    def create_behavior_tree(self, waypoints_in_range, destination):
        self.get_logger().info("Initializing Behavior Tree with new waypoints and destination")
        root = py_trees.composites.Sequence("RootSequence", memory=True)
        navigate_action = NavigateThroughPosesAction("NavigateThroughPosesAction", self.action_client, waypoints_in_range, destination)
        root.add_child(navigate_action)
        self.tree = py_trees.trees.BehaviourTree(root)
        self.tree.setup(timeout=5)
        self.tree.tick_tock(period_ms=100)  # 100ms마다 트리 tick

    # RViz에 마커 퍼블리시
    def publish_markers(self, waypoints, destination):
        self.get_logger().info("Entered publish_markers function")  # 함수 진입 로그
        marker_array = MarkerArray()  # 마커 배열 생성
        
        # 경유지 마커 생성 및 추가
        for i, point in enumerate(waypoints):
            # 경유지 위치 마커
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point['x']
            marker.pose.position.y = point['y']
            marker.pose.position.z = 0.1
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
            
            # 경유지 번호 표시 마커
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.id = i + len(waypoints)  # 고유 ID로 설정
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = point['x']
            text_marker.pose.position.y = point['y']
            text_marker.pose.position.z = 0.3  # 텍스트 마커를 약간 높여서 표시
            text_marker.scale.z = 0.2  # 텍스트 크기
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = str(i + 1)  # 경유지 번호
            marker_array.markers.append(text_marker)
            self.get_logger().info(f"Created marker for waypoint {i} at ({point['x']}, {point['y']})")  # 각 경유지 마커 로그
        
        # 목적지 마커 생성 및 추가
        destination_marker = Marker()
        destination_marker.header.frame_id = "map"
        destination_marker.id = len(waypoints) * 2
        destination_marker.type = Marker.SPHERE
        destination_marker.action = Marker.ADD
        destination_marker.pose.position.x = destination['x']
        destination_marker.pose.position.y = destination['y']
        destination_marker.pose.position.z = 0.1
        destination_marker.scale.x = 0.3
        destination_marker.scale.y = 0.3
        destination_marker.scale.z = 0.3
        destination_marker.color.a = 1.0
        destination_marker.color.r = 1.0
        destination_marker.color.g = 0.0
        destination_marker.color.b = 0.0
        marker_array.markers.append(destination_marker)
        self.get_logger().info(f"Created destination marker at ({destination['x']}, {destination['y']})")  # 목적지 마커 로그
        
        # MarkerArray 퍼블리시
        self.marker_pub.publish(marker_array)
        self.get_logger().info("Published markers for waypoints and destination")

def main(args=None):
    # ROS 2 초기화
    rclpy.init(args=args)

    # 명령줄 인수를 통해 YAML 경로를 받아와서 노드 생성
    if len(sys.argv) < 2:
        print("Usage: ros2 run navigate_waypoints navigate_with_bt <yaml_path>")
        return

    yaml_path = sys.argv[1]
    node = NavigateTree(yaml_path=yaml_path)  # NavigateTree 노드 생성
    rclpy.spin(node)  # 노드 실행
    node.destroy_node()  # 노드 종료 시 삭제
    rclpy.shutdown()  # ROS 2 종료

if __name__ == '__main__':
    main()
