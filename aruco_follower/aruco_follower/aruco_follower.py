import threading
import socket
import struct
import numpy as np
import cv2
import time
import queue  # 큐 모듈 임포트
import pickle
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import py_trees
from action_msgs.msg import GoalStatusArray  # GoalStatusArray 메시지 타입 임포트
import math
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

MAX_DGRAM = 65507  # UDP의 최대 패킷 크기
UDP_PORT1 = 9999   # 첫 번째 카메라용 포트
UDP_PORT2 = 9998   # 두 번째 카메라용 포트

# ArUco 사전 설정 (여기서는 DICT_4X4_50 사용)
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

# ArUco 마커 실제 크기 (52mm = 0.052m)
marker_length = 0.052  # 단위: 미터

# 캘리브레이션 데이터 로드
try:
    with open("/home/sehyung/minibot_ws/src/pinklab_minibot_robot/aruco_follower/aruco_follower/calibration_minibot5.pkl", 'rb') as f:
        calibration_data = pickle.load(f)
        camera_matrix = calibration_data["camera_matrix"]
        dist_coeffs = calibration_data["dist_coeffs"]
except Exception as e:
    print("Calibration file error:", e)

# 회전 행렬을 오일러 각도로 변환하는 함수 추가
def rotationMatrixToEulerAngles(R):
    sy = np.sqrt(R[0, 0] * R[0, 0] +  R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2,1] , R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

class SharedState:
    def __init__(self):
        self.goal_reached = False
        self.marker_detected = False
        self.marker_info = None
        self.lock = threading.Lock()

class WaitForNavGoalReached(py_trees.behaviour.Behaviour):
    def __init__(self, node, shared_state):
        super(WaitForNavGoalReached, self).__init__("WaitForNavGoalReached")
        self.node = node
        self.shared_state = shared_state
        self.last_log_time = time.time()  # 마지막 로그 시간을 저장

        # 상태 구독 추가
        self.node.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self.status_callback, 10
        )

    def setup(self, timeout=15):  # timeout을 선택적 인수로 변경
        return py_trees.common.Status.SUCCESS

    def status_callback(self, msg):
        # 상태가 3 또는 4일 때 goal_reached 플래그를 True로 설정
        with self.shared_state.lock:
            for status in msg.status_list:
                if status.status in (3, 4):  # 도착 또는 미도착 상태로 모두 True 처리
                    self.shared_state.goal_reached = True

    def update(self):
        with self.shared_state.lock:
            if self.shared_state.goal_reached:
                return py_trees.common.Status.SUCCESS
            else:
                # 1초마다 대기 중임을 출력
                if time.time() - self.last_log_time >= 1.0:
                    self.node.get_logger().info("Waiting for navigation goal to be reached...")
                    self.last_log_time = time.time()
                return py_trees.common.Status.RUNNING

class RotateToSearchMarker(py_trees.behaviour.Behaviour):
    def __init__(self, node, shared_state):
        super(RotateToSearchMarker, self).__init__("RotateToSearchMarker")
        self.node = node
        self.shared_state = shared_state
        self.publisher = self.node.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)

    def update(self):
        with self.shared_state.lock:
            if self.shared_state.marker_detected:
                return py_trees.common.Status.SUCCESS
            else:
                # 회전 명령어 발행
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.3  # 회전 속도 설정
                self.publisher.publish(twist)
                self.node.get_logger().info("No ArUco marker detected - Rotating to search")
                return py_trees.common.Status.RUNNING

class MoveTowardsMarker(py_trees.behaviour.Behaviour):
    def __init__(self, node, shared_state):
        super(MoveTowardsMarker, self).__init__("MoveTowardsMarker")
        self.node = node
        self.shared_state = shared_state
        self.publisher = self.node.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.desired_distance = 0.18  # 목표 거리 18cm
        self.distance_tolerance = 0.02  # 거리 허용 오차 ±2cm
        self.tolerance = 0.025  # 중심에서의 허용 오차 설정

    def update(self):
        with self.shared_state.lock:
            if not self.shared_state.marker_detected:
                return py_trees.common.Status.FAILURE
            else:
                distance = self.shared_state.marker_info['distance']
                x_offset = self.shared_state.marker_info['x_offset']
                distance_error = distance - self.desired_distance
                if abs(distance_error) > self.distance_tolerance:
                    # 이동 및 회전 명령어 발행
                    twist = Twist()
                    Kp_linear = 0.5  # 선속도 제어 이득
                    max_linear_speed = 0.16
                    twist.linear.x = Kp_linear * distance_error
                    twist.linear.x = max(-max_linear_speed, min(twist.linear.x, max_linear_speed))
                    # 각속도 제어
                    Kp_angular = 3.5  # 각속도 제어 이득
                    max_angular_speed = 0.8
                    twist.angular.z = -Kp_angular * x_offset
                    twist.angular.z = max(-max_angular_speed, min(twist.angular.z, max_angular_speed))
                    self.publisher.publish(twist)
                    self.node.get_logger().info(f"Moving towards marker (Distance Error: {distance_error:.2f} m, X Offset: {x_offset:.2f} m)")
                    return py_trees.common.Status.RUNNING
                else:
                    # 원하는 거리에 도달
                    self.node.get_logger().info("Desired distance achieved - Stopping movement")
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher.publish(twist)
                    return py_trees.common.Status.SUCCESS

class ReverseAndAdjustOrientation(py_trees.behaviour.Behaviour):
    def __init__(self, node, shared_state):
        super(ReverseAndAdjustOrientation, self).__init__("ReverseAndAdjustOrientation")
        self.node = node
        self.shared_state = shared_state
        self.publisher = self.node.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.state = 'initial'
        self.start_time = None
        self.reverse_duration = 3.5  # 후진 시간 (초)
        self.yaw_tolerance = 0.7  # 허용되는 Yaw 각도 오차 (도)
        self.desired_distance = 0.18  # 목표 거리 (MoveTowardsMarker와 동일)
        self.distance_tolerance = 0.02  # 거리 허용 오차
        self.tolerance = 0.025  # 중심에서의 허용 오차 설정

    def update(self):
        with self.shared_state.lock:
            if not self.shared_state.marker_detected:
                return py_trees.common.Status.FAILURE

            yaw = self.shared_state.marker_info['yaw']
            x_offset = self.shared_state.marker_info['x_offset']
            distance = self.shared_state.marker_info['distance']

            if self.state == 'initial':
                if abs(yaw) > self.yaw_tolerance:
                    self.state = 'reversing'
                    self.start_time = time.time()
                    self.node.get_logger().info(f"Starting reverse adjustment (Yaw: {yaw:.1f} degrees)")
                else:
                    # 자세 조정이 필요 없는 경우
                    return py_trees.common.Status.SUCCESS

            if self.state == 'reversing':
                elapsed_time = time.time() - self.start_time
                if elapsed_time < self.reverse_duration:
                    twist = Twist()
                    twist.linear.x = -0.085  # 후진 속도

                    # 각속도 제어
                    Kp_angular = 8.0
                    max_angular_speed = 8.0

                    yaw_error_rad = np.radians(yaw)
                    twist.angular.z = -Kp_angular * yaw_error_rad
                    twist.angular.z = max(-max_angular_speed, min(twist.angular.z, max_angular_speed))

                    self.publisher.publish(twist)
                    self.node.get_logger().info(f"Reversing to adjust orientation (Yaw: {yaw:.1f} degrees, Angular Z: {twist.angular.z:.2f})")
                    return py_trees.common.Status.RUNNING
                else:
                    self.state = 'forward'
                    self.node.get_logger().info("Reverse adjustment complete. Moving forward to realign.")
                    return py_trees.common.Status.RUNNING

            if self.state == 'forward':
                if not self.shared_state.marker_detected:
                    self.node.get_logger().info("Marker lost during forward adjustment.")
                    return py_trees.common.Status.FAILURE

                distance = self.shared_state.marker_info['distance']
                x_offset = self.shared_state.marker_info['x_offset']
                distance_error = distance - self.desired_distance

                if abs(distance_error) > self.distance_tolerance:
                    # 이동 및 회전 명령어 발행 (MoveTowardsMarker의 로직 사용)
                    twist = Twist()
                    Kp_linear = 0.5  # 선속도 제어 이득
                    max_linear_speed = 0.16
                    twist.linear.x = Kp_linear * distance_error
                    twist.linear.x = max(-max_linear_speed, min(twist.linear.x, max_linear_speed))
                    # 각속도 제어
                    Kp_angular = 7.0  # 각속도 제어 이득
                    max_angular_speed = 7.0
                    twist.angular.z = -Kp_angular * x_offset
                    twist.angular.z = max(-max_angular_speed, min(twist.angular.z, max_angular_speed))
                    self.publisher.publish(twist)
                    self.node.get_logger().info(f"Adjusting towards marker (Distance Error: {distance_error:.2f} m, X Offset: {x_offset:.2f} m)")
                    return py_trees.common.Status.RUNNING
                else:
                    # 원하는 거리에 도달
                    self.node.get_logger().info("Desired distance achieved during forward adjustment.")
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher.publish(twist)
                    self.state = 'done'
                    return py_trees.common.Status.RUNNING

            if self.state == 'done':
                # 로봇 멈춤
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher.publish(twist)
                return py_trees.common.Status.SUCCESS

class FineAlignToWall(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node, shared_state):
        super(FineAlignToWall, self).__init__("FineAlignToWall")
        self.node = node
        self.shared_state = shared_state
        self.publisher = self.node.create_publisher(
            Twist, '/base_controller/cmd_vel_unstamped', 10)

        # QoS 설정을 위한 프로파일 생성
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 또는 RELIABLE로 변경 가능
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # '/scan' 토픽을 QoS 설정과 함께 구독
        self.scan_subscription = self.node.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        self.node.get_logger().info("Subscribed to /scan topic with QoS settings.")

        self.laser_data = None
        self.angle_tolerance = 3.0  # 허용 오차 (도 단위)
        self.rotation_speed = 0.8  # 회전 속도 (rad/s)
        
        # alignment_complete 속성 초기화
        self.alignment_complete = False  # 정렬 완료 상태 초기화

    def scan_callback(self, msg):
        # LiDAR 데이터 수신 시 메시지 출력
        # self.node.get_logger().info("LiDAR data received in scan_callback.")
        self.laser_data = msg

    def calculate_wall_angle(self):
        if self.laser_data is None:
            self.node.get_logger().info("No LiDAR data available in calculate_wall_angle.")
            return None

        ranges = np.array(self.laser_data.ranges)
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment
        num_readings = len(ranges)
        angles = np.array([angle_min + i * angle_increment for i in range(num_readings)])
        angles_deg = np.degrees(angles)

        # 전방 -20도부터 20도 사이의 데이터 선택
        front_indices = np.where((angles_deg >= -20) & (angles_deg <= 20))[0]
        front_ranges = ranges[front_indices]
        front_angles_deg = angles_deg[front_indices]

        # 유효한 거리 데이터를 가진 포인트만 선택
        valid_indices = np.where(np.isfinite(front_ranges))[0]
        valid_ranges = front_ranges[valid_indices]
        valid_angles_deg = front_angles_deg[valid_indices]

        if len(valid_ranges) < 2:
            # 유효한 데이터가 충분하지 않음
            self.node.get_logger().info("Not enough valid LiDAR data to compute wall angle.")
            return None

        # x, y 좌표 계산
        x = valid_ranges * np.cos(np.radians(valid_angles_deg))
        y = valid_ranges * np.sin(np.radians(valid_angles_deg))

        # 직선 피팅 (최소제곱법)
        A = np.vstack([x, np.ones(len(x))]).T
        m, c = np.linalg.lstsq(A, y, rcond=None)[0]

        # 벽의 기울기 각도 계산
        wall_angle_rad = math.atan(m)
        wall_angle_deg = np.degrees(wall_angle_rad)
        self.node.get_logger().info(f"Calculated wall angle: {wall_angle_deg:.2f} degrees")

        return wall_angle_deg

    def update(self):
        if self.laser_data is None:
            self.node.get_logger().info("Waiting for LiDAR data in update...")
            return py_trees.common.Status.RUNNING

        wall_angle_deg = self.calculate_wall_angle()

        if wall_angle_deg is None:
            return py_trees.common.Status.RUNNING

        # 벽의 각도가 허용 오차 이내인지 확인
        if abs(wall_angle_deg) <= self.angle_tolerance:
            # 벽과 수직함
            twist = Twist()
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            self.publisher.publish(twist)
            self.node.get_logger().info("Fine alignment complete - Robot is perfectly perpendicular to the wall.")
            self.shared_state.goal_reached = False
            self.shared_state.marker_detected = False
            self.shared_state.marker_info = None
            return py_trees.common.Status.SUCCESS
        else:
            # 벽과 수직하지 않음, 회전 방향 결정
            twist = Twist()
            if wall_angle_deg > 0:
                # 벽이 왼쪽으로 기울어져 있음 => 시계 방향 회전
                twist.angular.z = -self.rotation_speed
            else:
                # 벽이 오른쪽으로 기울어져 있음 => 반시계 방향 회전
                twist.angular.z = self.rotation_speed
            twist.linear.x = 0.0
            self.publisher.publish(twist)
            self.node.get_logger().info(f"Fine aligning to wall (Wall Angle: {wall_angle_deg:.2f} degrees)")
            return py_trees.common.Status.RUNNING
    def update(self):
        # 이미 정렬이 완료된 경우, SUCCESS 상태를 유지
        if self.alignment_complete:
            return py_trees.common.Status.SUCCESS

        if self.laser_data is None:
            self.node.get_logger().info("Waiting for LiDAR data in update...")
            return py_trees.common.Status.RUNNING

        wall_angle_deg = self.calculate_wall_angle()

        if wall_angle_deg is None:
            return py_trees.common.Status.RUNNING

        # 벽의 각도가 허용 오차 이내인지 확인
        if abs(wall_angle_deg) <= self.angle_tolerance:
            # 벽과 수직함
            twist = Twist()
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            self.publisher.publish(twist)
            self.node.get_logger().info("Fine alignment complete - Robot is perfectly perpendicular to the wall.")
            
            # 완료 후 상태 설정
            self.alignment_complete = True  # 정렬 완료 상태로 변경
            return py_trees.common.Status.SUCCESS
        else:
            # 벽과 수직하지 않음, 회전 방향 결정
            twist = Twist()
            if wall_angle_deg > 0:
                # 벽이 왼쪽으로 기울어져 있음 => 시계 방향 회전
                twist.angular.z = -self.rotation_speed
            else:
                # 벽이 오른쪽으로 기울어져 있음 => 반시계 방향 회전
                twist.angular.z = self.rotation_speed
            twist.linear.x = 0.0
            self.publisher.publish(twist)
            self.node.get_logger().info(f"Fine aligning to wall (Wall Angle: {wall_angle_deg:.2f} degrees)")
            return py_trees.common.Status.RUNNING

                    
def create_behavior_tree(node, shared_state):
    root = py_trees.composites.Sequence("Root", memory=True)
    wait_for_nav_goal = WaitForNavGoalReached(node, shared_state)
    rotate_to_search_marker = RotateToSearchMarker(node, shared_state)
    move_towards_marker = MoveTowardsMarker(node, shared_state)
    reverse_and_adjust_orientation = ReverseAndAdjustOrientation(node, shared_state)  # Add the new node here
    align_to_marker = FineAlignToWall(node, shared_state)

    root.add_children([wait_for_nav_goal, rotate_to_search_marker, move_towards_marker, reverse_and_adjust_orientation, align_to_marker])

    return root

class FrameReceiver:
    def __init__(self, udp_port, shared_state):
        self.shared_state = shared_state

        # UDP 소켓 설정
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(('0.0.0.0', udp_port))
        self.frame_buffer = {}
        self.lock = threading.Lock()
        self.frame_queue = queue.Queue(maxsize=10)  # 프레임을 전달할 큐
        self.udp_port = udp_port  # 디버깅을 위한 포트 정보 저장

    def receive_data(self):
        while True:
            try:
                segment, addr = self.udp_socket.recvfrom(MAX_DGRAM)
                if len(segment) < 8:
                    continue  # 잘못된 패킷 무시

                # 헤더 파싱
                frame_id, total_chunks, seq_num, data_len, bg_num, br_code = struct.unpack('=HBBHBB', segment[:8])
                data = segment[8:]

                key = (addr, frame_id)
                with self.lock:
                    if key not in self.frame_buffer:
                        self.frame_buffer[key] = {
                            'total_chunks': total_chunks,
                            'chunks': [None] * total_chunks,
                            'received_chunks': 0,
                            'timestamp': time.time()
                        }

                    frame_info = self.frame_buffer[key]
                    if seq_num < total_chunks and frame_info['chunks'][seq_num] is None:
                        frame_info['chunks'][seq_num] = data
                        frame_info['received_chunks'] += 1

                    # 모든 청크를 수신한 경우 프레임 조립
                    if frame_info['received_chunks'] == frame_info['total_chunks']:
                        # 프레임 완성
                        frame_data = b''.join(frame_info['chunks'])
                        frame = np.frombuffer(frame_data, dtype=np.uint8)
                        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
                        if frame is not None:
                            # 왜곡 보정 적용
                            undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

                            # ArUco 마커 탐지 추가
                            gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
                            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

                            # 마커가 감지되었는지 확인
                            if ids is not None:
                                with self.shared_state.lock:
                                    self.shared_state.marker_detected = True
                                    # ArUco 마커가 감지되면 이동 및 회전하여 중앙에 위치
                                    for i in range(len(ids)):
                                        corner = corners[i]
                                        # 마커의 회전 및 변환 벡터를 추정
                                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, marker_length, camera_matrix, dist_coeffs)

                                        # rvec, tvec의 형태가 (1,1,3)이므로 (3,) 형태로 변경
                                        rvec_single = rvec[0][0]
                                        tvec_single = tvec[0][0]

                                        # 중심과의 오차 계산 (x 방향)
                                        x_offset = tvec_single[0]
                                        distance = tvec_single[2]  # 마커와의 거리 (z 방향)

                                        # 회전 벡터(rvec)를 회전 행렬로 변환
                                        rotation_matrix, _ = cv2.Rodrigues(rvec_single)
                                        # 회전 행렬을 오일러 각도로 변환
                                        angles = rotationMatrixToEulerAngles(rotation_matrix)
                                        roll, pitch, yaw = np.degrees(angles)

                                        # 정보를 shared_state에 저장
                                        self.shared_state.marker_info = {
                                            'x_offset': x_offset,
                                            'distance': distance,
                                            'roll': roll,
                                            'pitch': pitch,
                                            'yaw': yaw
                                        }

                                        # 마커의 위치와 방향을 그리기
                                        cv2.aruco.drawAxis(undistorted_frame, camera_matrix, dist_coeffs, rvec_single, tvec_single, 0.05)
                                        cv2.aruco.drawDetectedMarkers(undistorted_frame, corners)

                                        # 마커 ID, 거리, 각도 정보를 화면에 표시
                                        cX = int((corner[0][0][0] + corner[0][2][0]) / 2.0)
                                        cY = int((corner[0][0][1] + corner[0][2][1]) / 2.0)
                                        cv2.putText(undistorted_frame, f"ID: {ids[i][0]}", (cX, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                        cv2.putText(undistorted_frame, f"Dist: {distance:.2f} m", (cX, cY - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                                        cv2.putText(undistorted_frame, f"Angles (R,P,Y): ({roll:.1f}, {pitch:.1f}, {yaw:.1f})", (cX, cY + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                            else:
                                with self.shared_state.lock:
                                    self.shared_state.marker_detected = False


                            # 프레임 큐에 추가
                            if not self.frame_queue.full():
                                self.frame_queue.put(undistorted_frame)
                        else:
                            print(f"포트 {self.udp_port}: 프레임 디코딩 실패")
                        del self.frame_buffer[key]
                    else:
                        # 일정 시간 내에 모든 청크를 받지 못하면 해당 프레임 삭제
                        if time.time() - frame_info['timestamp'] > 1:  # 타임아웃 시간 1초로 조절
                            print(f"포트 {self.udp_port}: 프레임 {frame_id} 수신 시간 초과로 삭제합니다.")
                            del self.frame_buffer[key]

            except Exception as e:
                pass
                # print(f"포트 {self.udp_port}: 데이터 수신 중 오류 발생: {e}")

    def start_receiving(self):
        receive_thread = threading.Thread(target=self.receive_data, daemon=True)
        receive_thread.start()

class BehaviorTreeNode(Node):
    def __init__(self):
        super().__init__('aruco_behavior_tree_node')
        self.shared_state = SharedState()
        self.declare_parameter('camera_test', False)
        self.camera_test = self.get_parameter('camera_test').value
        print("camera_test parameter:", self.camera_test)

        # Behavior 초기화
        self.behavior_tree = create_behavior_tree(self, self.shared_state)
        self.tree_runner = py_trees.trees.BehaviourTree(self.behavior_tree)
        self.tree_runner.setup(timeout=15)  # setup 메서드 호출

        # FrameReceiver 시작
        self.receiver1 = FrameReceiver(UDP_PORT1, self.shared_state)
        self.receiver1_thread = threading.Thread(target=self.receiver1.receive_data)
        self.receiver1_thread.start()

        # Behavior Tree를 주기적으로 실행하는 타이머 생성
        self.create_timer(0.1, self.tick_tree)  # 0.1초마다 동작

        # camera_test가 True일 경우 프레임 표시를 위한 타이머 생성
        if self.camera_test:
            self.create_timer(0.03, self.display_frames)  # 약 30fps로 동작

    def tick_tree(self):
        self.tree_runner.tick()

    def display_frames(self):
        # print("Displaying frames...")
        # 프레임 큐에서 프레임을 가져와 표시
        try:
            frame = self.receiver1.frame_queue.get_nowait()
            cv2.imshow(f"Camera {self.receiver1.udp_port} - Undistorted with ArUco", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()
                cv2.destroyAllWindows()
        except queue.Empty:
            pass  # 큐가 비어있으면 넘어감

    def destroy_node(self):
        super().destroy_node()
        self.receiver1.udp_socket.close()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
