class StateMachine(Node):
    def __init__(self):
        super().__init__('googeese_control_pkg')

        # 초기 상태 설정
        self.state = 'Idle'
        self.get_logger().info(f'Initial State: {self.state}')

        # 콜백 그룹 설정
        
        self.driving_group = ReentrantCallbackGroup()

        # 주행 모드용 타이머 설정
        self.driving_timer = self.create_timer(
            DRIVING_CALLBACK_TIME_INTERVAL,
            self.driving_callback,
            callback_group=self.driving_group    
        )
        self.driving_timer.cancel()  # 초기에는 비활성화

        # cmd_vel 토픽 구독 및 퍼블리셔 설정
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/base_controller/cmd_vel_unstamped',
            self.cmd_vel_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/base_controller/cmd_vel_unstamped',
            10
        )

        # AI 서버와의 통신을 위한 큐 생성
        self.send_ai_q = queue.Queue(maxsize=10)
        self.recv_ai_q = queue.Queue(maxsize=10)

        # AI 서버 통신 객체 생성
        self.ai_server_connect = TCPConnect(
            AI_HOST,
            AI_PORT,
            self.send_ai_q, 
            self.recv_ai_q, 
            AI_TIME_INTERVAL
        )

        # 딥러닝 결과를 저장할 변수
        self.dl_data = None

        # 상태 전환을 위한 명령 수신을 위한 구독자 설정 (예: 'command' 토픽)
        self.command_sub = self.create_subscription(
            String,
            'command',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        """
        명령 토픽에서 상태 전환 명령을 수신하여 상태를 변경합니다.
        """
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        if self.state == 'Idle' and command == 'start':
            self.transition_to('Moving')
        elif command == 'stop':
            self.transition_to('Idle')
        elif command == 'error':
            self.transition_to('Error')
        elif self.state == 'Error' and command == 'recover':
            self.transition_to('Idle')
        else:
            self.get_logger().warn(f'Invalid command "{command}" in state "{self.state}"')

    def transition_to(self, new_state):
        """
        상태 전환을 수행하는 함수
        """
        self.get_logger().info(f'Transition: {self.state} → {new_state}')
        self.state = new_state

        # 상태에 따라 타이머 활성화/비활성화
        if new_state == 'Moving':
            self.driving_timer.reset()
            self.driving_timer.resume()
        else:
            self.driving_timer.cancel()

        # AI 서버에 새로운 상태를 전송
        self.send_ai_q.put({'state': self.state})

    def cmd_vel_callback(self, msg):
        """
        cmd_vel 토픽에서 Twist 메시지를 수신했을 때 호출되는 콜백 함수
        """
        self.get_logger().info(
            f'Received cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}'
        )

        if self.state == 'Moving':
            # 딥러닝 결과를 반영하여 cmd_vel 수정
            adjusted_cmd_vel = self.adjust_cmd_vel(msg)
            self.cmd_vel_pub.publish(adjusted_cmd_vel)
            self.get_logger().info(
                f'Published adjusted cmd_vel: linear.x={adjusted_cmd_vel.linear.x}, angular.z={adjusted_cmd_vel.angular.z}'
            )
        else:
            # 비주행 상태에서는 수신된 cmd_vel을 그대로 퍼블리시
            self.cmd_vel_pub.publish(msg)

    def adjust_cmd_vel(self, cmd_vel):
        """
        딥러닝 결과를 기반으로 cmd_vel 값을 조정하는 함수
        """
        adjusted_cmd_vel = Twist()
        adjusted_cmd_vel.linear = cmd_vel.linear
        adjusted_cmd_vel.angular = cmd_vel.angular

        # 딥러닝 결과가 있으면 cmd_vel 조정
        if self.dl_data:
            try:
                data = self.dl_data
                # 예: 딥러닝 결과에 따라 linear.x와 angular.z를 조정
                # 여기서는 간단히 딥러닝 결과를 사용하여 속도를 조정
                adjusted_cmd_vel.linear.x *= data.get('speed_factor', 1.0)
                adjusted_cmd_vel.angular.z += data.get('angular_adjustment', 0.0)
            except Exception as e:
                self.get_logger().error(f'Error adjusting cmd_vel: {e}')
        return adjusted_cmd_vel

    def driving_callback(self):
        """
        주행 모드에서 주기적으로 호출되는 콜백 함수
        """
        self.get_logger().info('Driving mode active.')

        # AI 서버로부터 딥러닝 결과 수신
        if not self.recv_ai_q.empty():
            self.dl_data = self.recv_ai_q.get()
            self.get_logger().info(f'Received DL data: {self.dl_data}')
        else:
            self.dl_data = None

        # 필요한 경우 추가적인 주행 로직 수행

    def destroy_node(self):
        """
        노드가 파괴될 때 타이머를 취소하는 함수
        """
        super().destroy_node()
        self.driving_timer.cancel()
