import queue
import json
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from googeese_control_pkg.tcp_connection import TCPConnect

GOOGEESE_GUI_HOST = '172.25.70.196'
GOOGEESE_GUI_PORT = 8889
TIME_INTERVAL = 0.5

ROBOT_ID = 5

class CommandNode(Node):
    """
    Googeesebot command 를 state machine 으로 관리
    """

    def __init__(self):
        super().__init__('googeese_control_pkg')

        self.response = {"robot_id": ROBOT_ID, "state" : ""}
        self.state = "start"
        self.get_logger().info(f'Initial State: {self.state}')

        self.cmd_request_pub = self.create_publisher(
            String,
            'Request',
            10
        )

        # 0.5 초마다 user request 를 확인하여 있는 경우, cmd_request 토픽 발행
        self.request_timer = self.create_timer(
            TIME_INTERVAL, 
            self.req_timer_callback
            )

        self.cmd_response_sub = self.create_subscription(
            String,
            'Response',
            self.response_callback,
            10
        )

        self.send_gui_server_q = queue.Queue(maxsize=10)
        self.recv_gui_server_q = queue.Queue(maxsize=10)

        # User GUI server 통신 객체 생성
        # 내부적으로 송신/수신 스레드를 각각 생성하여 비동기로 각각의
        # 연관 큐에 데이터를 넣고 빼서 작업을 진행
        self.GUI_server_connect = TCPConnect(
            GOOGEESE_GUI_HOST, 
            GOOGEESE_GUI_PORT, 
            self.send_gui_server_q, 
            self.recv_gui_server_q, 
            TIME_INTERVAL
            )
        
    def req_timer_callback(self):
        """
        GUI 로 부터 (TPC/IP) 통신으로 유저 요청 (상태 전환 명령)을 수신 하여 
        이를 command topic 발행
        주행/cargo 관련 노드에서 받아서 작업하기 위함
        """
        if not self.recv_gui_server_q.empty():
            data = self.recv_gui_server_q.get()            

            if data["robot_id"] == ROBOT_ID:                
                if data:
                    req_msg = String()
                    req_msg.data = data["state"]
                    self.cmd_request_pub.publish(req_msg)
            else:
                self.get_logger().info(f"Request robot id {data['robot_id']} is not matched with system robot id ({ROBOT_ID})")

    def response_callback(self, res_msg):
        """
        주행/cargo 관련 노드로 부터 현재 googeese bot 의 상태 정보를 받음
        이를 통해 GUI 동작 정보를 제공 (TCP/IP)
        """
        self.response["state"] = res_msg.data
        self.send_gui_server_q.put(self.response)

def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()