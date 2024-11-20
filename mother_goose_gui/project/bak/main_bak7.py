import sys
import multiprocessing
import threading
import os
import rclpy
from rclpy.node import Node
import yaml
from PySide2 import QtGui
from PySide2.QtCore import Qt, QTimer, QObject, Signal
from PySide2.QtGui import QColor, QPixmap, QPainter, QPen, QBrush
from PySide2.QtWidgets import QApplication, QMainWindow, QSizePolicy, QVBoxLayout
from ui_interface import Ui_MainWindow
from Custom_Widgets.Widgets import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from std_msgs.msg import String
from minibot_interfaces.msg import RobotState


def robot_monitor_process(robot_name, robot_id, status_queue, position_queue, motor_status_queue, lidar_status_queue, mode_state_queue, shutdown_event):
    # 도메인 ID 설정
    os.environ['ROS_DOMAIN_ID'] = str(robot_id)
    print(f"[{robot_name} Process] ROS_DOMAIN_ID set to {robot_id}")

    # ROS2 초기화
    rclpy.init(args=None)
    print(f"[{robot_name} Process] ROS2 initialized")

    # 노드 정의
    class RobotConnectionNode(Node):
        def __init__(self, robot_name, status_queue, position_queue, motor_status_queue, lidar_status_queue, mode_state_queue):
            super().__init__(f'{robot_name}_connection_node')
            self.robot_name = robot_name
            self.robot_id = robot_id
            # 로봇 on/off 상태
            self.status_queue = status_queue
            # 로봇 위치 좌표 구독
            self.position_queue = position_queue
            # 모터 상태 구독
            self.motor_status_queue = motor_status_queue
            # 라이다 상태 구독 
            self.lidar_status_queue = lidar_status_queue
            # 로봇 모드 상태 구독
            self.mode_state_queue = mode_state_queue
            self.connected = False
            # 상태 메세지 구독
            self.subscription_status = self.create_subscription(
                Odometry,
                '/base_controller/odom',
                self.odom_callback,
                10)
            self.subscription_position = self.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose',
                self.pose_callback,
                10)
            self.subscription_motor_status = self.create_subscription(
                RobotState,
                '/minibot_io_controller/robot_state',
                self.motor_status_callback,
                10
            )
            self.subscription_lidar_status = self.create_subscription(
                Range,
                '/minibot_io_controller/range',
                self.lidar_status_callback,
                10
            )
            self.subscription_state = self.create_subscription(
                String,
                '/mode_state',
                self.mode_state_callback,
                10
            )
            self.timer = self.create_timer(1.0, self.check_connection)
            self.last_msg_time = self.get_clock().now()
            print(f"[{self.robot_name}] Connection node initialized.")

        def mode_state_callback(self, msg):
            mode_state = msg.data.strip()
            parts = mode_state.split(' ', 1)
            if len(parts) ==2:
                robot_id_str, mode_state_str = parts
                try:
                    robot_id = int(robot_id_str)
                    if robot_id == self.robot_id:
                        self.mode_state_queue.put((self.robot_name, mode_state_str))
                        print(f"[{self.robot_name}] Received state: {robot_id}, {mode_state_str}")
                except ValueError:
                    print(f"Invalid robot_id: {robot_id_str}")
            else:
                print(f"Invalid mode_state format: {mode_state}")
            # self.mode_state_queue.put((self.robot_name, mode_state))
            # print(f"[{self.robot_name}] Received state: {mode_state}")

        def lidar_status_callback(self, msg):
            lidar_status = msg.range
            self.lidar_status_queue.put((self.robot_name, lidar_status))
            print(f"[{self.robot_name}] Received lidar status: {msg.range}")

        def motor_status_callback(self, msg):
            motor_status = msg.enable_motor
            self.motor_status_queue.put((self.robot_name, motor_status))
            print(f"[{self.robot_name}] Received motor status: {msg.enable_motor}")

        def odom_callback(self, msg):
            self.last_msg_time = self.get_clock().now()
            # print(f"[{self.robot_name}] Received odometry message.")
            if not self.connected:
                self.connected = True
                self.status_queue.put((self.robot_name, True))

        def pose_callback(self, msg):
            try:
                position = msg.pose.pose.position
                x = position.x
                y = position.y
                print(f"[{self.robot_name}] Received position: x={x}, y={y}")
                # 로봇의 이름과 위치를 함께 전송
                self.position_queue.put((self.robot_name, x, y))
            except Exception as e:
                print(f"[{self.robot_name}] Exception in pose_callback: {e}")

        def check_connection(self):
            time_since_last_msg = self.get_clock().now() - self.last_msg_time
            if time_since_last_msg.nanoseconds > 2e9 and self.connected:
                print(f"[{self.robot_name}] Connection lost.")
                self.connected = False
                self.status_queue.put((self.robot_name, False))

    # 노드 생성 및 스피닝
    node = RobotConnectionNode(robot_name, status_queue, position_queue, motor_status_queue, lidar_status_queue, mode_state_queue)
    print(f"[{robot_name} Process] Node created.")

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        ########################################################################
        # APPLY JSON STYLESHEET
        ########################################################################
        loadJsonStyle(self, self.ui)

        ########################################################################
        # 2:1 비율 적용 코드 추가
        layout = self.ui.centralwidget.findChild(QVBoxLayout, "verticalLayout_22")
        if layout:
            layout.setStretch(0, 2)  # 첫 번째 아이템의 stretch 값을 2로 설정
            layout.setStretch(1, 1)  # 두 번째 아이템의 stretch 값을 1로 설정

        ########################################################################
        # frame_14에 실시간 웹캠 영상 표시 설정
        ########################################################################
        # self.image_label = QLabel(self.ui.frame_14)  # QLabel 생성
        # self.image_label.setAlignment(Qt.AlignCenter)
        # self.ui.frame_14.layout().addWidget(self.image_label)  # frame_14에 QLabel 추가

        # self.cap = cv2.VideoCapture(0)  # 웹캠 열기 (0번 장치)

        # # QTimer 설정: 30ms마다 update_frame 호출하여 영상 업데이트
        # self.timer = QTimer()
        # self.timer.timeout.connect(self.update_frame)
        # self.timer.start(30)

        ########################################################################
        # 메뉴 확장 및 축소 기능 설정
        ########################################################################
        # EXPAND CENTER MENU WIDGET SIZE
        self.ui.settingsBtn.clicked.connect(lambda: self.ui.centerMenuContainer.expandMenu())
        self.ui.infoBtn.clicked.connect(lambda: self.ui.centerMenuContainer.expandMenu())
        self.ui.helpBtn.clicked.connect(lambda: self.ui.centerMenuContainer.expandMenu())
        
        # CLOSE CENTER MENU WIDGET
        self.ui.closeCenterMenuBtn.clicked.connect(lambda: self.ui.centerMenuContainer.collapseMenu())

        # EXPAND RIGHT MENU WIDGET SIZE
        self.ui.profileMenuBtn.clicked.connect(lambda: self.ui.rightMenuContainer.expandMenu())
        self.ui.moreMenuBtn.clicked.connect(lambda: self.ui.rightMenuContainer.expandMenu())
        
        # CLOSE RIGHT MENU WIDGET
        self.ui.closeRightMenuBtn.clicked.connect(lambda: self.ui.rightMenuContainer.collapseMenu())

        # CLOSE NOTIFICATION MENU WIDGET
        self.ui.closeNotificationBtn.clicked.connect(lambda: self.ui.popupNotificationContainer.collapseMenu())

        ########################################################################
        # ROS2 초기화 및 퍼블리셔 설정
        ########################################################################
        rclpy.init(args=None)
        self.ros_node = rclpy.create_node('gui_publisher_node')

        # Emergency Stop 퍼블리셔 생성
        self.emergency_publisher = self.ros_node.create_publisher(
            String,
            '/emergency_stop',
            10
        )

        # ROS2 스레드 시작
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True)
        self.ros_thread.start()

        # Emergency 버튼 클릭 이벤트
        self.ui.emergencyBtn_b1.clicked.connect(lambda: self.send_emergency_stop('robot1'))
        self.ui.emergencyBtn_b2.clicked.connect(lambda: self.send_emergency_stop('robot2'))


        ########################################################################
        # 맵 이미지 로드
        ########################################################################
        self.map_pixmap = QPixmap('/home/jook/minibot_ws/maps/map_4th_rev08_modified_gui_rev01.pgm')
        # self.map_pixmap = self.map_pixmap.scaled(392, 172, Qt.KeepAspectRatio)
        # self.ui.label_12.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # 원본 맵 이미지 크기 저장
        self.original_map_width = self.map_pixmap.width()
        self.original_map_height = self.map_pixmap.height()

        # 맵 이미지를 label_12에 설정
        self.ui.label_12.setPixmap(self.map_pixmap)

        # 맵 메타데이터 로드
        with open('/home/jook/minibot_ws/maps/map_4th_rev08_modified_gui_rev01.yaml', 'r') as file:
            map_metadata = yaml.safe_load(file)
        self.resolution = map_metadata['resolution']
        self.origin = map_metadata['origin']

        self.show()
        ########################################################################
        # 각 로봇 이름과 ID, 인디케이터 레이블 딕셔너리 정의
        ########################################################################
        self.robot_info = {
            'robot1': {
                'robot_id': 63, 
                'label' : [self.ui.label_status_b1, self.ui.label_mt_b1, self.ui.label_li_b1],
                'mode_state_label': self.ui.label_mode_state_b1
            },
            'robot2': {
                'robot_id': 77,
                'label' : [self.ui.label_status_b2, self.ui.label_mt_b2, self.ui.label_li_b2],
                'mode_state_label': self.ui.label_mode_state_b2
            }
        }

        # 상태 큐 및 위치 큐 생성
        self.status_queue = multiprocessing.Queue()
        self.position_queue = multiprocessing.Queue()
        self.motor_status_queue = multiprocessing.Queue()
        self.lidar_status_queue = multiprocessing.Queue()
        self.mode_state_queue = multiprocessing.Queue()

        # 로봇 위치를 저자할 딕셔너리 초기화
        self.robot_positions = {}

        # 프로세스 생성 및 시작
        self.processes = []
        for robot_name, info in self.robot_info.items():
            print(f"Starting process for {robot_name} with domain ID {info['robot_id']}")
            p = multiprocessing.Process(
                target=robot_monitor_process,
                args=(robot_name, info['robot_id'], self.status_queue, self.position_queue, self.motor_status_queue, self.lidar_status_queue, self.mode_state_queue),
                daemon=True)
            p.start()
            self.processes.append(p)

        # 상태 업데이트 타이머 설정
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.check_status_queue)
        self.status_timer.start(100)

        # 위치 업데이트 타이머 설정
        self.position_timer = QTimer()
        self.position_timer.timeout.connect(self.check_position_queue)
        self.position_timer.start(100)

        # 모터 업데이트 타이머 설정
        self.motor_status_timer = QTimer()
        self.motor_status_timer.timeout.connect(self.check_motor_status_queue)
        self.motor_status_timer.start(100)

        # 라이다 업데이트 타이머 설정
        self.lidar_status_timer = QTimer()
        self.lidar_status_timer.timeout.connect(self.check_lidar_status_queue)
        self.lidar_status_timer.start(100)

        # 모드 상태 업데이트 타이머 설정
        self.mode_state_timer = QTimer()
        self.mode_state_timer.timeout.connect(self.check_mode_state_queue)
        self.mode_state_timer.start(100)

    def send_emergency_stop(self, robot_name):
        if robot_name in self.robot_info:
            robot_id = self.robot_info[robot_name]['robot_id']
            msg = String()
            msg.data = f"STOP {robot_id}"
            self.emergency_publisher.publish(msg)
            print(f"Emergency stop sent to {robot_name} (ID: {robot_id})")
            self.ui.popupNotificationContainer.expandMenu()
            self.ui.label_13.setText(f"Emergency stop {robot_name} (ID: {robot_id})")
        else:
            print(f"Unknown robot: {robot_name}")

    def check_mode_state_queue(self):
        while not self.mode_state_queue.empty():
            robot_name, mode_state = self.mode_state_queue.get()
            # robot_name = self.get_robot_name_by_id(robot_id)
            if robot_name in self.robot_info:
                print(f"[Main Process] received mode state from {robot_name}: {mode_state}")
                self.on_mode_state_updated(robot_name, mode_state)
            else:
                print(f"Unknown robot ID: {robot_name}")
    
    def on_mode_state_updated(self, robot_name, state):
        font = mode_state_label.font()
        font.setBold(True)
        mode_state_label.setFont(font)
        if robot_name in self.robot_info:
            display_state = state.replace('_', ' ')
            mode_state_label = self.robot_info[robot_name]['mode_state_label']
            mode_state_label.setText(f"{display_state}")
            self.ui.popupNotificationContainer.expandMenu()
            self.ui.label_13.setText(f"{robot_name}'s mode is {display_state}")
        else:
            print(f"Unknown robot: {robot_name}")

    def check_lidar_status_queue(self):
        while not self.lidar_status_queue.empty():
            robot_name, lidar_status = self.lidar_status_queue.get()
            print(f"[Main Process] Received lidar status from {robot_name}: {lidar_status}")
            self.on_lidar_status_updated(robot_name, lidar_status)
    
    def on_lidar_status_updated(self, robot_name, lidar_status):
        if robot_name in self.robot_info:
            label = self.robot_info[robot_name]['label']
            if lidar_status is None:
                self.set_indicator_color(label[2], "red")
                print(f"{robot_name}'s lidar is malfunctioned")
                self.ui.popupNotificationContainer.expandMenu()
                self.ui.label_13.setText(f"{robot_name}'s lidar is malfunctioned")
            else:
                self.set_indicator_color(label[2], "green")

    def check_motor_status_queue(self):
        while not self.motor_status_queue.empty():
            robot_name, motor_status = self.motor_status_queue.get()
            print(f"[Main Process] Received motor status from {robot_name}: {motor_status}")
            self.on_motor_status_updated(robot_name, motor_status)

    def on_motor_status_updated(self, robot_name, motor_status):
        if robot_name in self.robot_info:
            label = self.robot_info[robot_name]['label']
            if motor_status is False:
                self.set_indicator_color(label[1], "red")
                print(f"{robot_name}'s motors are malfunctioned")
                self.ui.popupNotificationContainer.expandMenu()
                self.ui.label_13.setText(f"{robot_name}'s motors are malfunctioned")
            else:
                self.set_indicator_color(label[1], "green")

    def check_status_queue(self):
        while not self.status_queue.empty():
            robot_name, is_connected = self.status_queue.get()
            print(f"[Main Process] Received status update from {robot_name}: {'Connected' if is_connected else 'Disconnected'}")
            self.on_connection_status_updated(robot_name, is_connected)

    def on_connection_status_updated(self, robot_name, is_connected):
        if robot_name in self.robot_info:
            label = self.robot_info[robot_name]['label']
            if is_connected:
                self.set_indicator_color(label[0], "green")
                self.ui.popupNotificationContainer.expandMenu()
                self.ui.label_13.setText(f"{robot_name} is connected")
            else:
                self.set_indicator_color(label[0], "gray")
                self.ui.popupNotificationContainer.expandMenu()
                self.ui.label_13.setText(f"{robot_name} is disconnected")

    def set_indicator_color(self, label, color):
        label.setStyleSheet(f"""
        QLabel {{
            background-color: {color};
            border-radius: 10px;
        }}
        """)
 
    def check_position_queue(self):
        while not self.position_queue.empty():
            robot_name, x, y = self.position_queue.get()
            print(f"[Main Process] Received position from {robot_name}: x={x}, y={y}")
            # 로봇의 위치를 업데이트
            self.robot_positions[robot_name] = (x, y)
        # 위치 업데이트 후 맵 다시 그리기
        self.update_display()

    def update_display(self):
        self.draw_robot()

    def world_to_map(self, x, y, map_width, map_height):
        # 원본 맵 크기에 대한 비율 계산
        scale_x = map_width / self.original_map_width
        scale_y = map_height / self.original_map_height

        # 월드 좌표를 맵 픽셀 좌표로 변환
        map_x = int((x - self.origin[0]) / self.resolution * scale_x)
        map_y = int((y - self.origin[1]) / self.resolution * scale_y)

        # Y축 뒤집기 (이미지 좌표계와 월드 좌표계의 차이 고려)
        map_y = map_height - map_y

        return map_x, map_y

    def draw_robot(self):
        # 현재 label_12의 크기 가져오기
        label_width = self.ui.label_12.width()
        label_height = self.ui.label_12.height()

        # 맵 이미지를 label_12의 크기에 맞게 스케일링
        scaled_map = self.map_pixmap.scaled(label_width, label_height, Qt.KeepAspectRatio)

        # 로봇 위치 그리기
        updated_map = scaled_map.copy()

        # QPainter 객체 생성
        painter = QtGui.QPainter()
        # QPainter 시작
        painter.begin(updated_map)

        for robot_name, (x, y) in self.robot_positions.items():
            # 로봇 색상 설정 (원하는 대로 변경 가능)
            if robot_name == 'robot1':
                color = QColor(255, 0, 0)  # 빨간색
            elif robot_name == 'robot2':
                color = QColor(0, 255, 0)  # 초록색
            elif robot_name == 'robot3':
                color = QColor(0, 0, 255)  # 파란색
            else:
                color = QColor(0, 0, 0)  # 검은색

            pen = QPen(color)
            brush = QBrush(color)
            painter.setPen(pen)
            painter.setBrush(brush)

            # 월드 좌표를 맵 이미지의 픽셀 좌표로 변환
            map_x, map_y = self.world_to_map(x, y, updated_map.width(), updated_map.height())
            print(f'Drawing {robot_name} at map coordinates: x={map_x}, y={map_y}')

            robot_radius = 5  # 필요에 따라 조절
            painter.drawEllipse(map_x - robot_radius, map_y - robot_radius, robot_radius * 2, robot_radius * 2)
            print(f"Robot {robot_name} drawn")

        painter.end()

        # 업데이트된 이미지를 label_12에 설정
        self.ui.label_12.setPixmap(updated_map)

    def resizeEvent(self, event):
        self.update_display()
        super(MainWindow, self).resizeEvent(event)

    def closeEvent(self, event):
        # 종료 이벤트 설정
        self.shutdown_event.set()

        # 프로세스 종료
        for p in self.processes:
            p.join()

        # 타이머 중지
        self.status_timer.stop()
        self.position_timer.stop()
        self.motor_status_timer.stop()
        self.lidar_status_timer.stop()
        self.mode_state_timer.stop()

        # ROS2 노드 종료
        self.ros_node.destroy_node()
        rclpy.shutdown()
        self.ros_thread.join()

        event.accept()

# Execute App
if __name__ == "__main__":
    multiprocessing.set_start_method('spawn')
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec_())