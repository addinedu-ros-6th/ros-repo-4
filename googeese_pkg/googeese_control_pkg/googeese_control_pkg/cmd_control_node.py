import time
import rclpy
import queue
import socket
import threading

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TwistStamped

from googeese_control_pkg.tcp_connection import TCPConnect

AI_HOST = '127.0.0.1'
AI_PORT = 8888

AI_TIME_INTERVAL = 0.05
DRIVING_CALLBACK_TIME_INTERVAL = 0.05
RESPONSE_TIME_INTERVAL = 0.1

INITIAL_BODY_SIZE = 4000    # 해당 크기일 때 속도가 0이 되어야
MIN_BODY_SIZE = 3000        # 해당 크기일 때 속도가 최대 
FOLLOW_SIZE_FACTOR_RATIO = INITIAL_BODY_SIZE - MIN_BODY_SIZE
FOLLOW_TURN_FACTOR_RATIO = 180

MAX_VELOCITY = 1.0
MIN_VELOCITY = 0.0

MAX_ANGULAR = 1.0  # 방향은 모르겠음
MIN_ANGULAR = -1.0

class CmdControlNode(Node):
    def __init__(self):
        super().__init__('googeese_control_pkg')

        self.send_ai_q = queue.Queue(maxsize=10)
        self.recv_ai_q = queue.Queue(maxsize=10)

        # 초기 상태 (우선은 tracking 인식 대기 상태)
        self.state = 'idle' # 현재는 idle -> follow
        self.get_logger().info(f'Initial State: {self.state}')

        # AI server 통신 객체 생성
        # 내부적으로 송신/수신 스레드를 각각 생성하여 비동기로 각각의
        # 연관 큐에 데이터를 넣고 빼서 작업을 진행
        self.ai_server_connect = TCPConnect(
            AI_HOST, 
            AI_PORT, 
            self.send_ai_q, 
            self.recv_ai_q, 
            AI_TIME_INTERVAL
            )

        # GUI로부터 요청을 받는 '/Request' 토픽 구독
        self.request_subscriber = self.create_subscription(
            String,
            '/Request',
            self.request_callback,
            10)

        # GUI로 응답을 보내는 '/Response' 토픽 발행
        self.response_publisher = self.create_publisher(
            String,
            '/Response',
            10)
        
        self.timer_res_callback = self.create_timer(
            RESPONSE_TIME_INTERVAL,
            self.response_callback,
        )

        # 로봇의 현재 속도를 구독하는 '/base_controller/cmd_vel_out' 토픽 구독
        self.cmd_vel_out_subscriber = self.create_subscription(
            TwistStamped,
            '/base_controller/cmd_vel_out',
            self.cmd_vel_out_callback,
            10)  

        # 로봇의 속도를 제어하기 위한 '/base_controller/cmd_vel_unstamped' 토픽 발행
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/base_controller/cmd_vel_unstamped',
            10)

        # 상태 변수 초기화
        self.state = 'idle'
        self.cur_x_vel = 0
        self.cur_z_ang = 0

        self.follow_sub_mode = 0
        self.follow_diff_x = 0
        self.follow_body_size = 0
        self.follow_hand = "Others"


        # self.send_data = {
        #     "state": "idle",
        #     "camera": 1,             # 1: front 2: rear
        #     "follow":{            # 오타 수정
        #         "sub_mode": 0,       # 0: stop & searching 1: too far 2: move to left 3: move to right 4: following 5: pause
        #         "diff_x": 9999,
        #         "diff_y": 9999,
        #         "body_size": 0,
        #         "hand": "Others",
        #     },
        #     "aruco": {
        #         "detected": 0,
        #         "class": 0,         # 추후 정의 필요
        #         "conf": 0.0,
        #         "bbox_x": 0,
        #         "bbox_y": 0,
        #         "bbox_width": 0,
        #         "bbox_height": 0, 
        #     },
        # }

        self.pause_msg_sent = False
        threading.Thread(target=self.recv_ai_data, daemon=True).start()

    def response_callback(self):
        msg = String()
        # self.get_logger().info(f"state: {self.state}, hand :{self.follow_hand}")
        if self.state == "follow" and self.follow_hand == "pause":
            msg.data = "follow pause"
            self.get_logger().info(f"response_msg : {msg.data}")
            self.response_publisher.publish(msg)

            self.pause_msg_sent = True
            
            # 손 상태 값을 기본값으로 reset
            self.follow_hand = "others"



    def request_callback(self, msg):
        new_state = msg.data
        self.get_logger().info(f"state : {self.state} -> {new_state}")
        self.state = new_state

        # 상태가 변경될 경우 플래그 초기화
        if new_state != self.state:
            self.pause_msg_sent = False
    
        # AI 서버에 새로운 상태를 전송
        self.send_ai_q.put({"state": self.state}) 

    def cmd_vel_out_callback(self, msg):
        """ 현재 로봇의 속도 정보를 처리 (필요한 경우)"""
        try:
            # self.get_logger().info(f"{self.state}")
            # self.get_logger().info(f"{msg}")
            if self.state == "follow":
                # 딥러닝 결과를 반영하여 cmd_vel update
                updated_cmd_vel = self.update_cmd_vel(msg)
                self.cmd_vel_publisher.publish(updated_cmd_vel) 
            elif self.state == "follow_pause":
                # 속도/각속도 == 0 publish
                pause_cmd_vel = self.zero_vel()
                self.cmd_vel_publisher.publish(pause_cmd_vel)
            else:
                pass
                #TODO: 다른 모드들 추가 
        except Exception as e:
            self.get_logger().error(f"Error in cmd_vel_out_callback: {e}")

    def update_cmd_vel(self, cmd_vel_out):
        """
        딥러닝 결과를 기반으로 cmd_vel 값을 조정
        follow 고도화를 위해 해당 로직 추가 구현 필요
        """
        updated_cmd_vel = Twist()
        updated_cmd_vel.linear = cmd_vel_out.twist.linear
        updated_cmd_vel.angular = cmd_vel_out.twist.angular

        if self.follow_sub_mode == 4 and self.follow_hand != "Pause":
            velocity = 0 
            angular = 0
            if self.follow_body_size < INITIAL_BODY_SIZE:
                velocity = INITIAL_BODY_SIZE/self.follow_body_size * 0.15
            else :
                velocity = 0.0
            if self.follow_diff_x > 130:
                angular = -(self.follow_diff_x*0.5/250)
            elif self.follow_diff_x < -130:
                angular = -(self.follow_diff_x*0.5/250)
            else :
                angular = 0.0
            updated_cmd_vel.linear.x = velocity
            updated_cmd_vel.angular.z = angular
        else: 
            updated_cmd_vel.linear.x = 0.0
            updated_cmd_vel.angular.z = 0.0
        
        return updated_cmd_vel
    
    def zero_vel(self):
        """비주행 또는 pause 상태"""
        pause_cmd_vel = Twist()
        pause_cmd_vel.linear.x = 0.0
        pause_cmd_vel.linear.y = 0.0
        pause_cmd_vel.linear.z = 0.0
        pause_cmd_vel.angular.x = 0.0
        pause_cmd_vel.angular.y = 0.0
        pause_cmd_vel.angular.z = 0.0
        return pause_cmd_vel
    
    def destroy_node(self):
        if self.driving_timer is not None:
            self.driving_timer.cancel()
            self.driving_timer = None
        super().destroy_node()
    
    def recv_ai_data(self):
        """AI server 분석 및 cmd_vel 에서 필요한 정보를 지속적으로 갱신"""
        while rclpy.ok():
            if not self.recv_ai_q.empty():
                data = self.recv_ai_q.get()

                # self.get_logger().info(f"data: {data}")

                if self.state == "follow":
                    self.follow_sub_mode = data["follow"]["sub_mode"]
                    self.follow_diff_x = data["follow"]["diff_x"] 
                    self.follow_body_size = data["follow"]["body_size"]
                    self.follow_hand = data["follow"]["hand"]
                
                if self.state == "follow_pause":
                    self.follow_sub_mode = 0
                    self.follow_diff_x = 0 
                    self.follow_body_size = 0
                    self.follow_hand = "others"

                elif self.state == "auto_delivery":
                    pass
                    # TODO 
            else:
                time.sleep(0.05)


    def destroy_node(self):
        if self.driving_timer is not None:
            self.driving_timer.cancel()
            self.driving_timer = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
