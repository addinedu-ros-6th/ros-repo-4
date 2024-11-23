import rclpy
import yaml
import math
import py_trees
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
import action_msgs.msg
import tf2_ros
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped
from nav2_msgs.action import NavigateToPose
from shapely.geometry import LineString, Point
from builtin_interfaces.msg import Duration 


# === PlanPathClient 클래스 정의 ===
class PlanPathClient:
    def __init__(self, node: Node):
        self.node = node
        self.action_client = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
        self.path = None
        self.goal_sent = False

    def send_goal(self, start_pose: PoseStamped, goal_pose: PoseStamped):
        if self.goal_sent:
            self.node.get_logger().warn('이미 목표가 전송되었습니다.')
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start_pose
        goal_msg.goal = goal_pose

        self.node.get_logger().info('경로 계획 요청을 전송합니다...')
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.goal_sent = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('경로 계획 목표가 거부되었습니다.')
            self.path = None
            self.goal_sent = False
            return

        self.node.get_logger().info('경로 계획 목표가 수락되었습니다. 결과를 기다리는 중...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('경로 계획이 성공적으로 완료되었습니다!')
            self.node.get_logger().info(f"계획된 경로 포인트 수: {len(result.path.poses)}")
            self.path = result.path
            self.node.path = self.path
            self.node.visualize_path_range()  # 경로 주변의 포함 범위를 시각화
        else:
            self.node.get_logger().info(f'경로 계획이 실패했습니다. 상태 코드: {status}')
            self.path = None
        self.goal_sent = False
        self.node.plan_received = True
        self.node.tick_tree()

    def feedback_callback(self, feedback):
        pass

# === Behavior Tree 노드 정의 ===
class CheckGoalReceived(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(CheckGoalReceived, self).__init__(name)
        self.node = node

    def update(self):
        if self.node.goal_received:
            self.node.get_logger().info("목표가 수신되었습니다.")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

class CheckPlanReceived(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(CheckPlanReceived, self).__init__(name)
        self.node = node

    def update(self):
        if self.node.plan_received and self.node.path is not None:
            self.node.get_logger().info("경로가 수신되었습니다.")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

class LoadWaypoints(py_trees.behaviour.Behaviour):
    def __init__(self, name, yaml_path, node):
        super(LoadWaypoints, self).__init__(name)
        self.yaml_path = yaml_path
        self.node = node

    def initialise(self):
        self.node.load_points_from_yaml(self.yaml_path)
        self.node.get_logger().info("경유지와 목표 지점 로드 완료.")

    def update(self):
        return py_trees.common.Status.SUCCESS

class SetDestination(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(SetDestination, self).__init__(name)
        self.node = node

    def initialise(self):
        self.node.set_destination()
        self.node.get_logger().info("목적지 설정 완료.")

    def update(self):
        return py_trees.common.Status.SUCCESS

class PlanPathAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, planner_client, node):
        super(PlanPathAction, self).__init__(name)
        self.planner_client = planner_client
        self.node = node

    def initialise(self):
        if self.node.start_pose is None or self.node.destination_pose is None:
            self.node.get_logger().error("시작 포즈 또는 목적지 포즈가 설정되지 않았습니다.")
            self.status = py_trees.common.Status.FAILURE
            return
        self.planner_client.send_goal(self.node.start_pose, self.node.destination_pose)
        self.node.get_logger().info("경로 계획 요청을 전송했습니다.")

    def update(self):
        if self.node.plan_received and self.node.path is not None:
            self.node.get_logger().info("경로 계획이 완료되었습니다.")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

class FilterWaypoints(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(FilterWaypoints, self).__init__(name)
        self.node = node

    def initialise(self):
        self.node.filter_waypoints_near_path()

    def update(self):
        return py_trees.common.Status.SUCCESS

class VisualizeWaypoints(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(VisualizeWaypoints, self).__init__(name)
        self.node = node

    def initialise(self):
        self.node.visualize_filtered_waypoints()

    def update(self):
        return py_trees.common.Status.SUCCESS

class NavigateToWaypointAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, waypoint, index):
        super(NavigateToWaypointAction, self).__init__(name)
        self.node = node
        self.waypoint = waypoint
        self.index = index
        self.action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.goal_sent = False
        self.status = py_trees.common.Status.RUNNING

    def initialise(self):
        if not self.goal_sent:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = self.waypoint['point']['x']
            goal_pose.pose.position.y = self.waypoint['point']['y']
            goal_pose.pose.orientation.w = 1.0

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose

            self.node.get_logger().info(f"경유지 {self.index + 1}로 이동 시작: {self.waypoint}")
            self.action_client.wait_for_server()
            self._send_goal_future = self.action_client.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            self.goal_sent = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info(f"경유지 {self.index + 1}로 이동 목표가 거부되었습니다.")
            self.goal_sent = False
            self.status = py_trees.common.Status.FAILURE
            return

        self.node.get_logger().info(f"경유지 {self.index + 1}로 이동 목표가 수락되었습니다. 이동 중...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(f"경유지 {self.index + 1}에 성공적으로 도착했습니다.")
            self.node.update_markers(self.index)
            self.status = py_trees.common.Status.SUCCESS
            self.node.tick_tree()  # 다음 행동을 진행하기 위해 트리를 틱
        else:
            self.node.get_logger().info(f"경유지 {self.index + 1}로 이동 실패. 상태 코드: {status}")
            self.status = py_trees.common.Status.FAILURE

        self.goal_sent = False

    def feedback_callback(self, feedback):
        pass

    def update(self):
        return self.status

class NavigateWaypointsTree(Node):
    def __init__(self, yaml_path):
        super().__init__('navigate_waypoints_tree')
        self.yaml_path = yaml_path
        self.goal_received = False
        self.plan_received = False
        self.destination = None
        self.path = None
        self.completed_indices = []
        self.current_waypoint_index = 0
        self.filtered_marker_ids = []
        self.range_marker_ids = []
        self.start_pose = None
        self.destination_pose = None
        self.bt_setup = False  # Behavior Tree 설정 여부

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.planner_client = PlanPathClient(self)

        qos = QoSProfile(depth=10)
        self.goal_subscription = self.create_subscription(Int32, 'goal_id', self.goal_callback, qos)
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', qos)

        self.goal_points = []
        self.waypoints = []
        self.range_threshold = 0.5
        self.load_points_from_yaml(self.yaml_path)
        self.setup_behavior_tree()
        self.visualize_all_waypoints()
        # marker_publisher 초기화 추가
        self.marker_publisher = self.create_publisher(MarkerArray, 'filtered_waypoints_markers', 10)

        self.get_logger().info("NavigateWaypointsTree 노드가 초기화되었습니다.")

    def load_points_from_yaml(self, yaml_path):
        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                self.goal_points = data.get('goal_points', [])
                self.waypoints = data.get('waypoints', [])
                self.range_threshold = data.get('range_threshold', 0.5)
            self.get_logger().info(f'YAML에서 경유지와 목표를 성공적으로 로드했습니다: {self.waypoints}, {self.goal_points}')
        except Exception as e:
            self.get_logger().error(f"YAML 로드 실패: {e}")

    def goal_callback(self, msg):
        goal_id = msg.data
        self.destination = next((goal['point'] for goal in self.goal_points if goal['id'] == goal_id), None)

        if self.destination:
            self.get_logger().info(f"수신된 목표 ID {goal_id}. 목적지 설정: {self.destination}")
            self.goal_received = True
            self.tick_tree()
        else:
            self.get_logger().error(f"YAML에서 목표 ID {goal_id}를 찾을 수 없습니다.")

    def filter_waypoints_near_path(self):
        # 경로가 없으면 함수 종료
        if not self.path or not self.path.poses:
            self.get_logger().warn("경로가 없습니다. 경유지 필터링을 수행할 수 없습니다.")
            return

        self.get_logger().info("경로 기반으로 경유지를 필터링합니다...")

        # 필터링된 경유지를 저장할 리스트 초기화
        self.filtered_waypoints = []

        # 경로 주변의 경유지를 포함하는 버퍼 영역 생성
        path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in self.path.poses]
        path_line = LineString(path_points)
        buffer_distance = self.range_threshold  # 경로에서 경유지를 포함할 거리 설정
        path_buffer = path_line.buffer(buffer_distance)

        # 각 경유지를 확인하여 경로 주변에 있는지 판단
        for waypoint in self.waypoints:
            waypoint_point = Point(waypoint['point']['x'], waypoint['point']['y'])
            if path_buffer.contains(waypoint_point):
                self.filtered_waypoints.append(waypoint)

        self.get_logger().info(f"필터링된 경유지 수: {len(self.filtered_waypoints)}")
        self.visualize_filtered_waypoints()  # 필터링 후 시각화 호출

        # 경유지 행동 생성 및 트리에 추가
        waypoint_behaviors = []
        for i, waypoint in enumerate(self.filtered_waypoints):
            navigate_behavior = NavigateToWaypointAction(f"Navigate to {i+1}", self, waypoint, i)
            waypoint_behaviors.append(navigate_behavior)

        # 행동들을 순차적으로 실행하기 위한 Sequence 노드 생성
        waypoints_sequence = py_trees.composites.Sequence(name="Waypoints Sequence", memory=False)
        waypoints_sequence.add_children(waypoint_behaviors)

        # 기존의 트리에 추가하거나 교체
        if len(self.behaviour_tree.root.children) > 7:
            self.behaviour_tree.root.children[7] = waypoints_sequence  # 인덱스를 7로 변경하여 올바른 위치에 추가
        else:
            self.behaviour_tree.root.add_child(waypoints_sequence)

    def visualize_path_range(self):
        if not self.path:
            self.get_logger().warning("경로 데이터가 없어 경유지 범위를 시각화할 수 없습니다.")
            return

        marker_array = MarkerArray()
        for i, pose in enumerate(self.path.poses):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = 4000 + i
            marker.type = Marker.CYLINDER
            marker.pose.position = pose.pose.position
            marker.scale.x = marker.scale.y = self.range_threshold * 2
            marker.scale.z = 0.1
            marker.color.a = 0.3
            marker.color.r = 0.5
            marker.color.g = 0.0
            marker.color.b = 0.5
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        self.get_logger().info("경로 주변의 경유지 포함 범위가 시각화되었습니다.")

    def calculate_distance(self, point1, point2):
        return math.sqrt((point1['x'] - point2['x']) ** 2 + (point1['y'] - point2['y']) ** 2)

    def visualize_all_waypoints(self):
        marker_array = MarkerArray()
        for i, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = 1000 + i
            marker.type = Marker.SPHERE
            marker.pose.position.x = waypoint['point']['x']
            marker.pose.position.y = waypoint['point']['y']
            marker.pose.position.z = waypoint['point'].get('z', 0.0)
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.color.a = 0.5
            marker.color.b = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        self.get_logger().info("모든 경유지가 파란색으로 시각화되었습니다.")

    def visualize_filtered_waypoints(self):
        marker_array = MarkerArray()
        for i, waypoint in enumerate(self.filtered_waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = 2000 + i
            marker.type = Marker.SPHERE
            marker.pose.position.x = waypoint['point']['x']
            marker.pose.position.y = waypoint['point']['y']
            marker.pose.position.z = waypoint['point'].get('z', 0.0)
            marker.scale.x = marker.scale.y = marker.scale.z = 0.25
            marker.color.a = 1.0

            # 색상 설정
            if i in self.completed_indices:
                # 완료된 경유지: 녹색
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif i == self.current_waypoint_index:
                # 현재 목표 경유지: 빨간색
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                # 나머지 경유지: 파란색
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0

            marker.lifetime = Duration(sec=0, nanosec=0)
            marker_array.markers.append(marker)

            # 텍스트 마커 추가 (경유지 번호)
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.id = 3000 + i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.pose.position.x = waypoint['point']['x']
            text_marker.pose.position.y = waypoint['point']['y']
            text_marker.pose.position.z = waypoint['point'].get('z', 0.0) + 0.3
            text_marker.scale.z = 0.3
            text_marker.color.a = 1.0
            text_marker.color.r = text_marker.color.g = text_marker.color.b = 1.0
            text_marker.text = str(waypoint['id'])
            text_marker.lifetime = Duration(sec=0, nanosec=0)
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"필터링된 경유지가 시각화되었습니다. 퍼블리시된 마커 수: {len(marker_array.markers)}")

    def update_markers(self, completed_index):
        self.completed_indices.append(completed_index)
        self.current_waypoint_index += 1
        self.get_logger().info(f"마커 업데이트: 완료된 경유지 인덱스 {completed_index}")
        self.visualize_filtered_waypoints()

    def setup_behavior_tree(self):
        self.root = py_trees.composites.Sequence("Navigate Waypoints Sequence", memory=True)

        load_waypoints = LoadWaypoints("Load Waypoints", self.yaml_path, self)
        check_goal_received = CheckGoalReceived("Check Goal Received", self)
        set_destination = SetDestination("Set Destination", self)
        plan_path = PlanPathAction("Plan Path", self.planner_client, self)
        check_plan_received = CheckPlanReceived("Check Plan Received", self)
        filter_waypoints = FilterWaypoints("Filter Waypoints", self)
        visualize_waypoints = VisualizeWaypoints("Visualize Waypoints", self)

        self.navigate_sequence = py_trees.composites.Sequence("Navigate Through Waypoints", memory=True)

        self.root.add_children([
            load_waypoints,
            check_goal_received,
            set_destination,
            plan_path,
            check_plan_received,
            filter_waypoints,
            visualize_waypoints,
            self.navigate_sequence
        ])

        self.behaviour_tree = py_trees.trees.BehaviourTree(self.root)
        self.bt_setup = True
        self.get_logger().info("Behavior Tree가 설정되었습니다.")

    def tick_tree(self):
        if self.bt_setup and self.behaviour_tree is not None:
            self.behaviour_tree.tick()

    def main_loop(self):
        while rclpy.ok():
            self.tick_tree()
            rclpy.spin_once(self, timeout_sec=0.1)

    def set_destination(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.start_pose = PoseStamped()
            self.start_pose.header.frame_id = 'map'
            self.start_pose.pose.position.x = trans.transform.translation.x
            self.start_pose.pose.position.y = trans.transform.translation.y
            self.start_pose.pose.position.z = trans.transform.translation.z
            self.start_pose.pose.orientation = trans.transform.rotation
            self.get_logger().info("시작 포즈 설정 완료.")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"현재 위치를 가져오는 중 오류 발생: {e}")
            return

        self.destination_pose = PoseStamped()
        self.destination_pose.header.frame_id = 'map'
        self.destination_pose.pose.position.x = self.destination['x']
        self.destination_pose.pose.position.y = self.destination['y']
        self.destination_pose.pose.position.z = self.destination.get('z', 0.0)
        self.destination_pose.pose.orientation.w = 1.0
        self.get_logger().info(f"목적지 포즈 설정 완료: {self.destination}")

    # 이 메서드는 더 이상 사용되지 않으므로 삭제하거나 주석 처리할 수 있습니다.
    # def publish_filtered_waypoints_markers(self):
    #     pass

def main(args=None):
    rclpy.init(args=args)
    yaml_path = '/home/sehyung/minibot_ws/src/pinklab_minibot_robot/navigate_waypoints/navigate_waypoints/points.yaml'
    node = NavigateWaypointsTree(yaml_path=yaml_path)
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    try:
        node.main_loop()
    except KeyboardInterrupt:
        node.get_logger().info('키보드 인터럽트로 종료 요청됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
