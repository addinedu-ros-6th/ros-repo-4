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

MAX_DGRAM = 65507  # UDP의 최대 패킷 크기
UDP_PORT1 = 9999   # 첫 번째 카메라용 포트
UDP_PORT2 = 9998   # 두 번째 카메라용 포트

# ArUco 사전 설정 (여기서는 DICT_4X4_50 사용)
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

# ArUco 마커 실제 크기 (52mm = 0.052m)
marker_length = 0.052  # 단위: 미터

# 캘리브레이션 데이터 로드
with open("/home/sehyung/minibot_ws/src/pinklab_minibot_robot/aruco_follower/aruco_follower/calibration_minibot5.pkl", 'rb') as f:
    calibration_data = pickle.load(f)
    camera_matrix = calibration_data["camera_matrix"]
    dist_coeffs = calibration_data["dist_coeffs"]

class FrameReceiver(Node):
    def __init__(self, udp_port):
        super().__init__('aruco_follower')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)

        # UDP 소켓 설정
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(('0.0.0.0', udp_port))
        self.frame_buffer = {}
        self.lock = threading.Lock()
        self.frame_queue = queue.Queue()  # 프레임을 전달할 큐
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
                                # ArUco 마커가 감지되면 이동 및 회전하여 중앙에 위치
                                for i in range(len(ids)):
                                    corner = corners[i]
                                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, marker_length, camera_matrix, dist_coeffs)
                                    
                                    # 중심과의 오차 계산 (x 방향)
                                    x_offset = tvec[0][0][0]
                                    distance = tvec[0][0][2]  # 마커와의 거리 (z 방향)
                                    tolerance = 0.05  # 중심에서의 허용 오차 설정

                                    # 목표 거리 및 허용 오차 설정
                                    desired_distance = 0.1  # 목표 거리 0.1m (10cm)
                                    distance_tolerance = 0.02  # 거리 허용 오차 ±2cm

                                    # 제어기 이득 설정
                                    Kp_linear = 0.5  # 선속도 제어 이득
                                    Kp_angular_base = 3.5  # 기본 각속도 제어 이득

                                    # 로봇의 크기 (폭) 고려하여 각속도 이득 조절
                                    robot_width = 0.2  # 로봇의 폭 20cm
                                    Kp_angular_max = 5.0  # 각속도 이득 최대값
                                    Kp_angular_min = 1.0  # 각속도 이득 최소값

                                    # 최대 속도 제한
                                    max_linear_speed = 0.2  # 최대 선속도
                                    max_angular_speed = 1.0  # 최대 각속도

                                    twist = Twist()

                                    # 거리 오차 계산
                                    distance_error = distance - desired_distance

                                    # 거리 비율 계산 (멀수록 1보다 큰 값)
                                    distance_ratio = distance / desired_distance

                                    # 각속도 이득 조절 (멀수록 더 크게 회전하도록)
                                    Kp_angular = min(Kp_angular_base * distance_ratio, Kp_angular_max)
                                    Kp_angular = max(Kp_angular, Kp_angular_min)

                                    # 거리 오차에 따른 선속도 조절
                                    if abs(distance_error) > distance_tolerance:
                                        twist.linear.x = Kp_linear * distance_error
                                        # 선속도 제한
                                        twist.linear.x = max(-max_linear_speed, min(twist.linear.x, max_linear_speed))
                                        self.get_logger().info(f"Moving towards marker (Distance Error: {distance_error:.2f} m)")
                                    else:
                                        twist.linear.x = 0.0
                                        self.get_logger().info("Desired distance achieved - Stopping linear movement")

                                    # 중심 오차에 따른 각속도 조절 (로봇의 폭 고려)
                                    desired_x_offset = 0.0  # 목표 중심 오프셋 (0이면 카메라 중앙)
                                    x_offset_error = x_offset - desired_x_offset

                                    twist.angular.z = -Kp_angular * x_offset_error
                                    # 각속도 제한
                                    twist.angular.z = max(-max_angular_speed, min(twist.angular.z, max_angular_speed))

                                    if abs(x_offset_error) <= tolerance:
                                        self.get_logger().info("Marker centered - Stopping rotation")
                                    else:
                                        self.get_logger().info(f"Adjusting orientation (X Offset: {x_offset_error:.2f} m, Kp_angular: {Kp_angular:.2f})")

                                    # 명령어 발행
                                    self.cmd_vel_pub.publish(twist)

                                    # 마커의 위치와 방향을 그리기
                                    cv2.aruco.drawAxis(undistorted_frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
                                    cv2.aruco.drawDetectedMarkers(undistorted_frame, corners)

                                    # 마커 ID와 거리 표시
                                    cX = int((corner[0][0][0] + corner[0][2][0]) / 2.0)
                                    cY = int((corner[0][0][1] + corner[0][2][1]) / 2.0)
                                    cv2.putText(undistorted_frame, f"ID: {ids[i][0]}",
                                                (cX, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            else:
                                # 마커가 감지되지 않으면 회전하면서 탐색
                                twist = Twist()
                                twist.linear.x = 0.0
                                twist.angular.z = 0.3  # 회전 속도 설정
                                self.cmd_vel_pub.publish(twist)
                                self.get_logger().info("No ArUco marker detected - Rotating to search")

                            # 카메라 화면 표시
                            self.frame_queue.put(undistorted_frame)
                            cv2.imshow(f"Camera {self.udp_port} - Undistorted with ArUco", undistorted_frame)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break  # 'q' 키를 누르면 종료
                        else:
                            print(f"포트 {self.udp_port}: 프레임 디코딩 실패")
                        del self.frame_buffer[key]
                    else:
                        # 일정 시간 내에 모든 청크를 받지 못하면 해당 프레임 삭제
                        if time.time() - frame_info['timestamp'] > 1:  # 타임아웃 시간 1초로 조절
                            print(f"포트 {self.udp_port}: 프레임 {frame_id} 수신 시간 초과로 삭제합니다.")
                            del self.frame_buffer[key]

            except Exception as e:
                print(f"포트 {self.udp_port}: 데이터 수신 중 오류 발생: {e}")

    def start_receiving(self):
        receive_thread = threading.Thread(target=self.receive_data, daemon=True)
        receive_thread.start()

def main(args=None):
    rclpy.init(args=args)
    # 두 개의 프레임 수신기 생성
    receiver1 = FrameReceiver(UDP_PORT1)
    receiver2 = FrameReceiver(UDP_PORT2)

    # 수신 시작
    receiver1.start_receiving()
    receiver2.start_receiving()

    rclpy.spin(receiver1)

    # 종료 처리
    receiver1.udp_socket.close()
    receiver2.udp_socket.close()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
