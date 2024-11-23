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
        self.status = py_trees.common.Status.RUNNING

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
        elif self.node.plan_received and self.node.path is None:
            self.node.get_logger().error("경로 계획에 실패했습니다.")
            return py_trees.common.Status.FAILURE
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


class NavigateAndVisualizeWaypoints(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(NavigateAndVisualizeWaypoints, self).__init__(name)
        self.node = node
        self.current_index = 0
        self.status = py_trees.common.Status.INVALID
        self.action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.goal_sent = False
        self.final_goal_sent = False

    def initialise(self):
        self.current_index = 0
        self.status = py_trees.common.Status.RUNNING
        self.node.get_logger().info("경유지 이동 및 시각화 시작")
        self.goal_sent = False
        self.final_goal_sent = False

    def update(self):
        # 경유지 이동 처리
        if self.current_index < len(self.node.filtered_waypoints):
            if not self.goal_sent:
                waypoint = self.node.filtered_waypoints[self.current_index]
                is_final_goal = (self.current_index == len(self.node.filtered_waypoints) - 1)
                self.send_goal(waypoint, self.current_index, is_final_goal)
            return self.status

        # 최종 목적지에 도달한 경우
        elif not self.final_goal_sent:
            self.node.get_logger().info("모든 경유지 완료, 최종 목적지에 도착했습니다.")
            self.final_goal_sent = True
            self.node.reset_navigation()
            self.status = py_trees.common.Status.SUCCESS
            return self.status

        return self.status

    def send_goal(self, waypoint, index, is_final_goal):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_pose.pose.position.x = waypoint['point']['x']
        goal_pose.pose.position.y = waypoint['point']['y']

        if is_final_goal:
            # 최종 목적지: orientation을 명시적으로 설정
            goal_pose.pose.orientation.w = 1.0
        else:
            # 경유지: 현재 로봇 방향 유지
            try:
                trans = self.node.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                goal_pose.pose.orientation = trans.transform.rotation
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.node.get_logger().error(f"현재 로봇 방향을 가져오는 중 오류 발생: {e}")
                goal_pose.pose.orientation.w = 1.0  # 기본값 설정

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.node.get_logger().info(f"경유지 {index + 1}로 이동 시작: {waypoint}")
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.goal_sent = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info(f"경유지 {self.current_index + 1} 이동 목표가 거부되었습니다.")
            self.status = py_trees.common.Status.FAILURE
            self.goal_sent = False
            return

        self.node.get_logger().info(f"경유지 {self.current_index + 1} 이동 목표가 수락되었습니다.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(f"경유지 {self.current_index + 1}에 성공적으로 도착했습니다.")
            self.node.update_markers(self.current_index)
            self.current_index += 1
            self.goal_sent = False
            self.status = py_trees.common.Status.RUNNING
        else:
            self.node.get_logger().info(f"경유지 {self.current_index + 1}로 이동 실패")
            self.status = py_trees.common.Status.FAILURE
            self.goal_sent = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        self.node.get_logger().info(
            f"경유지 {self.current_index + 1}로 이동 중... 현재 위치: "
            f"x={current_pose.position.x:.2f}, y={current_pose.position.y:.2f}"
        )



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
        self.bt_setup = False

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
        else:
            self.get_logger().error(f"YAML에서 목표 ID {goal_id}를 찾을 수 없습니다.")

    def filter_waypoints_near_path(self):
        if not self.path or not self.path.poses:
            self.get_logger().warn("경로가 없습니다. 경유지 필터링을 수행할 수 없습니다.")
            return

        self.get_logger().info("경로 기반으로 경유지를 필터링합니다...")

        self.filtered_waypoints = []

        path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in self.path.poses]
        path_line = LineString(path_points)
        buffer_distance = self.range_threshold
        path_buffer = path_line.buffer(buffer_distance)

        for waypoint in self.waypoints:
            waypoint_point = Point(waypoint['point']['x'], waypoint['point']['y'])
            if path_buffer.contains(waypoint_point):
                self.filtered_waypoints.append(waypoint)

        # 최종 목적지 추가
        if self.destination:
            self.filtered_waypoints.append({'point': self.destination})

        self.get_logger().info(f"필터링된 경유지 수: {len(self.filtered_waypoints)}")
        self.visualize_filtered_waypoints()

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

            if i in self.completed_indices:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif i == self.current_waypoint_index:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0

            marker.lifetime = Duration(sec=0, nanosec=0)
            marker_array.markers.append(marker)

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
            text_marker.text = f"{i + 1}"  # 경유지 번호 표시
            text_marker.lifetime = Duration(sec=0, nanosec=0)
            marker_array.markers.append(text_marker)

        # 최종 목적지 시각화 추가
        if self.destination:
            final_marker = Marker()
            final_marker.header.frame_id = "map"
            final_marker.id = 5000  # 고유 ID 설정
            final_marker.type = Marker.SPHERE
            final_marker.pose.position.x = self.destination['x']
            final_marker.pose.position.y = self.destination['y']
            final_marker.pose.position.z = self.destination.get('z', 0.0)
            final_marker.scale.x = final_marker.scale.y = final_marker.scale.z = 0.3
            final_marker.color.a = 1.0
            final_marker.color.r = 1.0
            final_marker.color.g = 0.0
            final_marker.color.b = 0.0
            marker_array.markers.append(final_marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"필터링된 경유지와 최종 목적지가 시각화되었습니다. 퍼블리시된 마커 수: {len(marker_array.markers)}")

    def update_markers(self, completed_index):
        self.completed_indices.append(completed_index)
        self.current_waypoint_index += 1
        self.get_logger().info(f"마커 업데이트: 완료된 경유지 인덱스 {completed_index}")
        self.visualize_filtered_waypoints()

    def reset_navigation(self):
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

        # 시각화 제거
        self.marker_pub.publish(MarkerArray())
        self.get_logger().info("네비게이션 상태 초기화 완료.")

    def setup_behavior_tree(self):
        self.root = py_trees.composites.Sequence("Navigate Waypoints Sequence", memory=True)

        load_waypoints = LoadWaypoints("Load Waypoints", self.yaml_path, self)
        check_goal_received = CheckGoalReceived("Check Goal Received", self)
        set_destination = SetDestination("Set Destination", self)
        plan_path = PlanPathAction("Plan Path", self.planner_client, self)
        check_plan_received = CheckPlanReceived("Check Plan Received", self)
        filter_waypoints = FilterWaypoints("Filter Waypoints", self)
        navigate_and_visualize_waypoints = NavigateAndVisualizeWaypoints("Navigate and Visualize Waypoints", self)

        self.root.add_children([
            load_waypoints,
            check_goal_received,
            set_destination,
            plan_path,
            check_plan_received,
            filter_waypoints,
            navigate_and_visualize_waypoints
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