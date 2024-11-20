import sys
import cv2
import threading
import rclpy
from rclpy.node import Node
import yaml
from PySide2.QtCore import Qt, QTimer, QObject, Signal
from PySide2.QtGui import QImage, QPixmap
from PySide2.QtWidgets import QApplication, QLabel, QMainWindow, QSizePolicy
from ui_interface import Ui_MainWindow
from Custom_Widgets.Widgets import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class ConnectionStatusUpdater(QObject):
    connection_status_updated = Signal(bool)

class RobotConnectionNode(Node):
    def __init__(self, updater):
        super().__init__('robot_connection_node')
        self.updater = updater
        self.connected = False
        self.subscription = self.create_subscription(
            Odometry,
            '/base_controller/odom',
            self.odom_callback,
            10)
        self.timer = self.create_timer(1.0, self.check_connection)
        self.last_msg_time = self.get_clock().now()
    
    def odom_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        if not self.connected:
            self.connected = True
            self.updater.connection_status_updated.emit(True)

    def check_connection(self):
        time_since_last_msg = self.get_clock().now() - self.last_msg_time
        if time_since_last_msg.nanoseconds > 2e9 and self.connected:
            self.connected = False
            self.updater.connection_status_updated.emit(False)

class PositionUpdater(QObject):
    position_updated = Signal(float, float)

class RobotPositionNode(Node):
    def __init__(self, updater):
        super().__init__('robot_position_node')
        self.updater = updater
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning

    def pose_callback(self, msg):
        position = msg.pose.pose.position
        x = position.x
        y = position.y
        print(f'Received position: x={x}, y={y}')
        self.updater.position_updated.emit(x, y)

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

        self.show()

        ########################################################################
        # 맵 이미지 로드
        ########################################################################
        self.map_pixmap = QPixmap('/home/jook/minibot_ws/maps/map_4th_rev08_modified_gui.pgm')
        # self.map_pixmap = self.map_pixmap.scaled(500, 500, Qt.KeepAspectRatio)
        self.ui.label_12.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # 원본 맵 이미지 크기 저장
        self.original_map_width = self.map_pixmap.width()
        self.original_map_height = self.map_pixmap.height()

        # 맵 이미지를 label_12에 설정
        self.ui.label_12.setPixmap(self.map_pixmap)

        # 맵 메타데이터 로드
        with open('/home/jook/minibot_ws/maps/map_4th_rev08_modified_gui.yaml', 'r') as file:
            map_metadata = yaml.safe_load(file)
        self.resolution = map_metadata['resolution']
        self.origin = map_metadata['origin']

        # 로봇 위치 초기화
        self.robot_x = 0.0
        self.robot_y = 0.0

        # 위치 업데이트 설정
        self.position_updater = PositionUpdater()
        self.position_updater.position_updated.connect(self.on_position_updated)

        ########################################################################
        # 각 로봇 이름과 ID, 인디케이터 레이블 딕셔너리 정의
        ########################################################################
        self.robot_info = {
            'robot1': {'domain_id': 5, 'label' : self.ui.label_status_b1},
            'robot2': {'domain_id': 63, 'label' : self.ui.label_status_b2}
        }

        # ROS2 노드 생성 및 스레드 시작
        rclpy.init(args=None)
        self.ros_node = RobotPositionNode(self.position_updater)
        ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True)
        ros_thread.start()

        # 타이머 설정
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(100)

        # 연결 상태 업데이트 설정
        self.connection_updater = ConnectionStatusUpdater()
        self.connection_updater.connection_status_updated.connect(self.on_connection_status_updated)

        # ROS2 노드 생성 및 스레드 시작
        self.connection_node = RobotConnectionNode(self.connection_updater)
        connection_thread = threading.Thread(target=rclpy.spin, args=(self.connection_node, ), daemon=True)
        connection_thread.start()

    def on_connection_status_updated(self, is_connected):
        if is_connected:
            self.set_indicator_color("green")
        else:
            self.set_indicator_color("gray")

    def set_indicator_color(self, color):
        self.ui.label_20.setStyleSheet(f"""
         QLabel {{
            background-color: {color};
            border-radius: 10px;
         }}
         """)

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

    def draw_robot(self, x, y):
        # 현재 label_12의 크기 가져오기
        label_width = self.ui.label_12.width()
        label_height = self.ui.label_12.height()

        # 맵 이미지를 label_12의 크기에 맞게 스케일링
        scaled_map = self.map_pixmap.scaled(label_width, label_height, Qt.KeepAspectRatio)

        # 로봇 위치 그리기
        updated_map = scaled_map.copy()
        painter = QPainter(updated_map)
        pen = QPen(QColor(255, 0, 0))
        painter.setPen(pen)
        painter.setBrush(QColor(255, 0, 0))

        # 월드 좌표를 맵 이미지의 픽셀 좌표로 변환
        map_x, map_y = self.world_to_map(x, y, updated_map.width(), updated_map.height())
        print(f'Drawing robot at map coordinates: x={map_x}, y={map_y}')

        robot_radius = 5  # 필요에 따라 조절
        painter.drawEllipse(map_x - robot_radius, map_y - robot_radius, robot_radius * 2, robot_radius * 2)
        painter.end()

        # 업데이트된 이미지를 label_12에 설정
        self.ui.label_12.setPixmap(updated_map)

    def on_position_updated(self, x, y):
        self.robot_x = x
        self.robot_y = y
        print(f'Updated position: x={x}, y={y}')

    def update_display(self):
        self.draw_robot(self.robot_x, self.robot_y)

    def resizeEvent(self, event):
        self.update_display()
        super(MainWindow, self).resizeEvent(event)

    def closeEvent(self, event):
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()

# Execute App
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec_())