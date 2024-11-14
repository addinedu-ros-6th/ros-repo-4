import sys
import cv2
import threading
import rclpy
from rclpy.node import Node
import yaml
from PySide2.QtCore import Qt, QTimer, QObject, Signal
from PySide2.QtGui import QImage, QPixmap
from PySide2.QtWidgets import QApplication, QLabel, QMainWindow
from ui_interface import Ui_MainWindow
from Custom_Widgets.Widgets import *
from geometry_msgs.msg import PoseWithCovarianceStamped

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
        self.map_pixmap = self.map_pixmap.scaled(1000, 1000, Qt.KeepAspectRatio)
        self.ui.label_12.setPixmap(self.map_pixmap)

       # 맵 메타데이터 로드
        with open('/home/jook/minibot_ws/maps/map_4th_rev08_modified_gui.yaml', 'r') as file:
            map_metadata = yaml.safe_load(file)
        self.resolution = map_metadata['resolution']
        self.origin = map_metadata['origin']
        self.map_width = self.map_pixmap.width()
        self.map_height = self.map_pixmap.height()

        # 로봇 위치 초기화
        self.robot_x = 0.0
        self.robot_y = 0.0

        # 위치 업데이트 설정
        self.position_updater = PositionUpdater()
        self.position_updater.position_updated.connect(self.on_position_updated)

        # ROS2 노드 생성 및 스레드 시작
        rclpy.init(args=None)
        self.ros_node = RobotPositionNode(self.position_updater)
        ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True)
        ros_thread.start()

        # 타이머 설정
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(100)

    def world_to_map(self, x, y):
        map_x = int((x - self.origin[0]) / self.resolution)
        map_y = int((y - self.origin[1]) / self.resolution)
        map_y = self.map_height - map_y
        return map_x, map_y

    def draw_robot(self, x, y):
        map_x, map_y = self.world_to_map(x, y)
        updated_map = self.map_pixmap.copy()
        painter = QPainter(updated_map)
        pen = QPen(QColor(255, 0, 0))
        painter.setPen(pen)
        painter.setBrush(QColor(255, 0, 0))
        robot_radius = 5
        painter.drawEllipse(map_x - robot_radius, map_y - robot_radius, robot_radius * 2, robot_radius * 2)
        painter.end()
        self.ui.frame_14.setPixmap(updated_map)

    def on_position_updated(self, x, y):
        self.robot_x = x
        self.robot_y = y
        print(f'Updated position: x={x}, y={y}')


    def update_display(self):
        self.draw_robot(self.robot_x, self.robot_y)


    def closeEvent(self, event):
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()

    def update_frame(self):
        ret, frame = self.cap.read()  # 웹캠에서 프레임 읽기
        if not ret:
            return

        # OpenCV 이미지(BGR)를 Qt 이미지(RGB)로 변환
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qimage = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)

        # QLabel에 맞춰 크기 조절 및 픽스맵 설정
        pixmap = QPixmap.fromImage(qimage).scaled(
            self.image_label.width(), 
            self.image_label.height(), 
            Qt.KeepAspectRatio
        )
        self.image_label.setPixmap(pixmap)

    def closeEvent(self, event):
        # 프로그램 종료 시 웹캠 릴리즈
        self.cap.release()
        event.accept()



class RobotPositionNode(Node):
    def __init__(self, updater):
        super().__init__('robot_position_node')
        self.updater = updater
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)

    def pose_callback(self, msg):
        position = msg.pose.pose.position
        x = position.x
        y = position.y
        self.updater.position_updated.emit(x, y)

# Execute App
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
