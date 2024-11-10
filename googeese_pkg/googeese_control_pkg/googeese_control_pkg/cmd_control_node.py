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

AI_HOST = '192.168.1.16'
AI_PORT = 8888
AI_TIME_INTERVAL = 0.05
DRIVING_CALLBACK_TIME_INTERVAL = 0.05

INITIAL_BODY_SIZE = 4000    # 해당 크기일 때 속도가 0이 되어야
MIN_BODY_SIZE = 1500        # 해당 크기일 때 속도가 최대 
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

        # 초기 상태 (우선은 tracking 인식 대기 상태)
        self.state = 'Idle' # 현재는 Idle -> Follow
        self.get_logger().info(f'Initial State: {self.state}')
        
        # 콜백 그룹 설정
        # self.non_driving_group = MutuallyExclusiveCallbackGroup()
        self.driving_group = ReentrantCallbackGroup()               
        self.driving_timer = None  # 초기 생성 X

        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            '/base_controller/cmd_vel_out',
            self.cmd_vel_callback,
            10
            )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/base_controller/cmd_vel_unstamped',
            10
            )
        
        self.send_ai_q = queue.Queue(maxsize=10)
        self.recv_ai_q = queue.Queue(maxsize=10)

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

        self.ai_data = None

        self.command_sub = self.create_subscription(
            String,
            'command',
            self.command_callback,
            10
        )
        
        self.cur_x_vel = 0
        self.cur_z_ang = 0

        self.following_sub_mode = 0
        self.following_diff_x = 0
        self.following_body_size = 0
        

    def command_callback(self, msg):
        """
        명령 토픽에서 상태 전환 명령을 수신하여 상태를 변경합니다.
        """
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        if self.state == 'Idle' and command == 'Follow':
            self.transition_to('Follow')
        elif command == 'stop':
            pass
        elif command == 'error':
            pass
        elif self.state == 'Error' and command == 'recover':
            pass
        else:
            self.get_logger().warn(f'Invalid command "{command}" in state "{self.state}"')


    def transition_to(self, new_state):
        """
        상태 전환을 수행하는 함수
        """
        self.get_logger().info(f'Transition: {self.state} → {new_state}')
        self.state = new_state

        # 상태에 따라 타이머 활성화/비활성화
        if new_state == 'Follow':
            if self.driving_timer is None:
                self.driving_timer = self.create_timer(
                    DRIVING_CALLBACK_TIME_INTERVAL,
                    self.driving_callback,
                    callback_group=self.driving_group    
                )
        else:
            # 타이머 취소
            if self.driving_timer is not None:
                self.driving_timer.cancel()
                self.driving_timer = None

        # AI 서버에 새로운 상태를 전송
        self.send_ai_q.put({'operating_code': 3})

    def cmd_vel_callback(self, msg):
        """cmd_vel_out 토픽에서 TwistStemped 메시지를 수신했을 때 호출되는 콜백 함수"""
        self.get_logger().info(
            f'Received cmd_vel_out: linear.x={msg.twist.linear.x}, angular.z={msg.twist.angular.z}'
        )

        if self.state == "Follow":
            # 딥러닝 결과를 반영하여 cmd_vel update
            updated_cmd_vel = self.update_cmd_vel(msg)
            self.cmd_vel_pub.publish(updated_cmd_vel)
            self.get_logger().info(
                f'Published updated cmd_vel: linear.x={updated_cmd_vel.linear.x}, angular.z={updated_cmd_vel.angular.z}'
            )
        else:
            # 비주행 상태에서는 모든 속도를 0으로 설정한 Twist 메시지 발행
            zero_cmd_vel = Twist()
            zero_cmd_vel.linear.x = 0.0
            zero_cmd_vel.linear.y = 0.0
            zero_cmd_vel.linear.z = 0.0
            zero_cmd_vel.angular.x = 0.0
            zero_cmd_vel.angular.y = 0.0
            zero_cmd_vel.angular.z = 0.0

            self.cmd_vel_pub.publish(zero_cmd_vel)

    def update_cmd_vel(self, cmd_vel_out):
        """
        딥러닝 결과를 기반으로 cmd_vel 값을 조정
        Following 고도화를 위해 해당 로직 추가 구현 필요
        """
        updated_cmd_vel = Twist()
        updated_cmd_vel.linear = cmd_vel_out.twist.linear
        updated_cmd_vel.angular = cmd_vel_out.twist.angular

        if self.following_sub_mode == 4:
            velocity = (self.following_body_size - MIN_BODY_SIZE) / FOLLOW_SIZE_FACTOR_RATIO
            if velocity < MIN_VELOCITY:
                velocity = MIN_VELOCITY
            elif velocity > MAX_VELOCITY:
                velocity = MAX_VELOCITY

            angular = -self.following_diff_x / FOLLOW_TURN_FACTOR_RATIO
            if angular < MIN_ANGULAR:
                angular = MIN_ANGULAR
            elif angular > MAX_ANGULAR:
                angular = MAX_ANGULAR

            updated_cmd_vel.linear.x = velocity
            updated_cmd_vel.angular.z = angular
        else: 
            updated_cmd_vel.linear.x = 0.0
            updated_cmd_vel.angular.z = 0.0
        
        return updated_cmd_vel
    
    def driving_callback(self):
        """
        주행 모드에서 주기적으로 호출되는 콜백 함수
        """
        self.get_logger().info('Driving mode active.')

        # AI 서버로부터 딥러닝 결과 수신
        if not self.recv_ai_q.empty():
            data = self.recv_ai_q.get()
            self.following_sub_mode = data["following"]["sub_mode"]
            self.following_diff_x = data["following"]["diff_x"] 
            self.following_body_size = data["following"]["body_size"]
    
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