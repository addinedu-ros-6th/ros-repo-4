import RPi.GPIO as GPIO
from time import sleep
import spidev
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

servoPin = 12 #서보
SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3
start_degree = 0
open_door = 85

GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPin, GPIO.OUT)
GPIO.setwarnings(False)
GPIO.setup(6, GPIO.OUT) #LED 
GPIO.setup(13, GPIO.IN) #근접센서 

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 50000

servo = GPIO.PWM(servoPin, 50)
servo.start(0)

def read_spi_adc(adcChannel):
    buff = spi.xfer2([1, (8 + adcChannel) << 4, 0])
    adcValue = ((buff[1] & 3) << 8) + buff[2]
    return adcValue

class ProximitySensorPublisher(Node):
    def __init__(self):
        super().__init__('proximity_sensor_publisher')
        self.publisher_ = self.create_publisher(Int32, 'proximity', 10)
        self.door_state = self.create_publisher(Int32, 'door_state', 10)

        self.timer = self.create_timer(0.5, self.publish_proximity)

        self.servo_timer = None  # 서보 모터 타이머
        self.target_degree = start_degree  # 목표 각도
        self.current_degree = start_degree  # 현재 각도

        GPIO.output(6, False)
        
        self.last_digit_val = None  # 가장 최근 받은 digit_val 저장

        # 구독 설정 추가
        self.subscription = self.create_subscription(
            Int32,
            'control_servo',
            self.control_servo_callback,
            10
        ) 

    def publish_proximity(self):
        adcValue = read_spi_adc(0)
        digit_val = GPIO.input(13)
        self.last_digit_val = digit_val
            
        if digit_val == 1:                  #근접센서에 물체가 접근했을 때 
            GPIO.output(6, True)
            msg = Int32()               
            msg.data = 30                
            self.publisher_.publish(msg)    #30을 퍼블리싱
        else:
            GPIO.output(6, False)           #근접센서에서 물체가 멀리 있을 때 
            msg = Int32()
            msg.data = 40
            self.publisher_.publish(msg)    #40을 퍼블리싱 

    # 서보모터 제어 콜백 함수
    def control_servo_callback(self, msg):
        if msg.data == 1:                   #수신된 메시지 값이 1일 때 서보모터를 80도로 이동
            print("Give me your luggage!!")
            door = Int32()  
            door.data = 10                  #퍼블리싱
            self.door_state.publish(door)
            self.target_degree = open_door

        elif msg.data == 0:                 #토픽 수신
            print("I have your luggage!!")
            door = Int32()
            door.data = 20                  #퍼블리싱
            self.door_state.publish(door)
            self.target_degree = start_degree

        # 서보 모터 타이머 시작
        if self.servo_timer is None:
            self.servo_timer = self.create_timer(0.02, self.update_servo_position)

    # 서보 모터 위치 업데이트
    def update_servo_position(self):
        # 목표위치 도달 시 불필요한 움직임 멈춤
        if abs(self.current_degree - self.target_degree) <= 1:
            servo.ChangeDutyCycle(0)
            self.servo_timer.cancel()
            self.servo_timer = None
            return

        # 현재 위치를 목표 위치에 근접하게 변경
        if self.current_degree < self.target_degree:
            self.current_degree += 1
        elif self.current_degree > self.target_degree:
            self.current_degree -= 1

        # 각도(degree)를 duty로 변경
        duty = SERVO_MIN_DUTY + (self.current_degree * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180.0)
        servo.ChangeDutyCycle(duty)


def main(args=None):
    rclpy.init(args=args)
    node = ProximitySensorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        servo.stop()
        GPIO.cleanup()
        spi.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
