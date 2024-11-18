import queue
import json
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TwistStamped

from googeese_control_pkg.tcp_connection import TCPConnect

AI_HOST = '127.0.0.1'

AI_PORT = 8888
AI_TIME_INTERVAL = 0.05
DRIVING_CALLBACK_TIME_INTERVAL = 0.05

INITIAL_BODY_SIZE = 4000    # 해당 크기일 때 속도가 0이 되어야
MIN_BODY_SIZE = 3000        # 해당 크기일 때 속도가 최대 
FOLLOW_SIZE_FACTOR_RATIO = INITIAL_BODY_SIZE - MIN_BODY_SIZE
FOLLOW_TURN_FACTOR_RATIO = 180

MAX_VELOCITY = 1.0
MIN_VELOCITY = 0.0

MAX_ANGULAR = 1.0  # 방향은 모르겠음
MIN_ANGULAR = -1.0

class CmdControlNode(Node):
    """
    Googeesebot 의 주행 및 비주행 상태를 state machine 으로 관리
    """

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

        self.command_req_sub = self.create_subscription(
            String,
            'Request',
            self.command_req_sub_callback,
            10
        )

        self.command_res_pub = self.create_publisher(
            String,
            'Response',
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            '/base_controller/cmd_vel_out',
            self.cmd_vel_callback,
            10
            )
        
        # follow 모드에서만 사용
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/base_controller/cmd_vel_unstamped',
            10
            )

        self.cur_x_vel = 0
        self.cur_z_ang = 0

        self.following_sub_mode = 0
        self.following_diff_x = 0
        self.following_body_size = 0

        # 주행 (follow, guide, auto delivery mode 상태에서 주기적으로 cmd_vel pub/sub callback 을 실행하기 위한 그룹)
        self.driving_group = ReentrantCallbackGroup()               
        self.driving_timer = None  # 초기 생성 X
        
    def command_req_sub_callback(self, msg):
        """
        주행 모드 상태에서만 AI server 분석 결과를 실시간으로 가져오는 callback 이 활성화 되도록 함
        주행 -> 비주행 모드 전환이 필요한 경우, 1회성으로 AI server 에 상태 정보를 전달
        """
        request = msg.data
        self.get_logger().info(f'Transition: {self.state} → {request}')

        self.state = request
        if self.state == "follow":
            self.get_logger().info('Driving mode active.')
            if self.driving_timer is None:
                self.driving_timer = self.create_timer(
                    DRIVING_CALLBACK_TIME_INTERVAL,
                    self.driving_callback,
                    callback_group=self.driving_group    
                )

        else: # auto delivery mode 포함???????????????????
            # 주행 관련 callback 비활성화
            self.get_logger().info('Driving mode deactive.')
            if self.driving_timer is not None:
                self.driving_timer.cancel()
                self.driving_timer = None 

        # AI 서버에 새로운 상태를 전송
        self.send_ai_q.put({"state": self.state})                

    def driving_callback(self):
        """주행 모드 상태에서 주기적으로 AI server 분석 및 cmd_vel 을 직접 처리함."""
        if not self.recv_ai_q.empty():
            data = self.recv_ai_q.get()
            # 쌓여 있는 모든 큐 내용을 비우고 마지막 값을 가져와서 사용
            while not self.recv_ai_q.empty():
                data = self.recv_ai_q.get()

            # self.get_logger().info(f"data: {data}")
            
            if self.state == "follow":
                self.following_sub_mode = data["following"]["sub_mode"]
                self.following_diff_x = data["following"]["diff_x"] 
                self.following_body_size = data["following"]["body_size"]

                # pause 모드 전환은 우선 client GUI 에 보낸 이후, 다신 받아서 follow pause 진행
                if self.following_sub_mode == 5: # pause 
                    send_msg = String()
                    send_msg.data = "follow pause"
                    self.command_res_pub.publish(send_msg)
                    self.state = "follow pause"
                    self.following_sub_mode = 0
                    
            elif self.state == "auto_delivery":
                pass
                # TODO 


    def cmd_vel_callback(self, msg):
        """cmd_vel_out 토픽에서 TwistStemped 메시지를 수신했을 때 호출되는 콜백 함수"""
        # self.get_logger().info(
        #     f'Received cmd_vel_out: linear.x={msg.twist.linear.x}, angular.z={msg.twist.angular.z}'
        # )

        if self.state == "follow":
            # 딥러닝 결과를 반영하여 cmd_vel update
            updated_cmd_vel = self.update_cmd_vel(msg)
            self.cmd_vel_pub.publish(updated_cmd_vel)
            # self.get_logger().info(
            #     f'Published updated cmd_vel: linear.x={updated_cmd_vel.linear.x}, angular.z={updated_cmd_vel.angular.z}'
            # )
        elif self.state == "follow_pause":
            # 속도/각속도 == 0 publish
            self.zero_vel()

        # follow & follow_pause 가 아닌 상태의 경우 주행을 어떤방식으로 할지 결정되지 않았으므로
        # 아무것도 publish 하지 않음.

    def update_cmd_vel(self, cmd_vel_out):
        """
        딥러닝 결과를 기반으로 cmd_vel 값을 조정
        Following 고도화를 위해 해당 로직 추가 구현 필요
        """
        updated_cmd_vel = Twist()
        updated_cmd_vel.linear = cmd_vel_out.twist.linear
        updated_cmd_vel.angular = cmd_vel_out.twist.angular

        if self.following_sub_mode == 4:
            # velocity = (self.following_body_size - MIN_BODY_SIZE) / FOLLOW_SIZE_FACTOR_RATIO
            # if velocity < MIN_VELOCITY:
                # velocity = MIN_VELOCITY
            # elif velocity > MAX_VELOCITY:
                # velocity = MAX_VELOCITY

            # angular = -self.following_diff_x / FOLLOW_TURN_FACTOR_RATIO
            # if angular < MIN_ANGULAR:
                # angular = MIN_ANGULAR
            # elif angular > MAX_ANGULAR:
                # angular = MAX_ANGULAR
            velocity = 0 
            angular = 0

            if self.following_body_size < INITIAL_BODY_SIZE:
                velocity = INITIAL_BODY_SIZE/self.following_body_size * 0.15
            else :
                velocity = 0.0

            if self.following_diff_x > 130:
                angular = -(self.following_diff_x*0.5/250)
            elif self.following_diff_x < -130:
                angular = -(self.following_diff_x*0.5/250)
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
        zero_cmd_vel = Twist()
        zero_cmd_vel.linear.x = 0.0
        zero_cmd_vel.linear.y = 0.0
        zero_cmd_vel.linear.z = 0.0
        zero_cmd_vel.angular.x = 0.0
        zero_cmd_vel.angular.y = 0.0
        zero_cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(zero_cmd_vel)
    
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
