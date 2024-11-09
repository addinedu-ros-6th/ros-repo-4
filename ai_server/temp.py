
import socket
import threading
import sys
import json 
import cv2
import time
import queue
import logging
import pickle
from ultralytics import YOLO
from body_follower import *
from udp_connection import *
from aruco_identification import *

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 서버 설정
HOST = '192.168.1.16'  # 서버 IP 주소
PORT = 8888             # 사용할 포트 번호

FRONT_LANZ_CAL_PATH = "lanz_calibration/calibration_minibot5.pkl"
REAR_LANZ_CAL_PATH  = "lanz_calibration/calibration_data120.pkl" 

class FrameProcessor:
    def __init__(self, front_frame_queue, rear_frame_queue, front_display_queue, rear_display_queue, send_msg_queue):
        self.model = YOLO("yolov8s.pt")  # GPU 사용 설정
        self.front_frame_queue = front_frame_queue
        self.rear_frame_queue = rear_frame_queue
        self.front_display_queue = front_display_queue
        self.rear_display_queue = rear_display_queue
        self.send_msg_queue = send_msg_queue

        self.operation_code = 1
        self.operation_code_lock = threading.Lock()  # 스레드 안전을 위한 락
        self.body_follower = BodyFollower()
        self.running = True

        # 스레드 시작
        self.thread = threading.Thread(target=self.process_frames, daemon=True)
        self.thread.start()

        self.send_data = {
            "operating_code": 0,
            "camera": 1,             # 1: front 2: rear
            "following":{
                "sub_mode": 0,       # 0: stop & searching 1: too far 2: move to left 3: move to right 4: following 
                "diff_x": 9999,
                "diff_y": 9999,
                "body_size": 0,
            },
            "obstacle": {
                "detected": 0,
                "class": 0,         # 추후 정의 필요
                "conf": 0.0,
                "bbox_x": 0,
                "bbox_y": 0,
                "bbox_width": 0,
                "bbox_height": 0, 
            },
            "face": {
                "conf" : 0.0,
            }
        }

    def update_operation_code(self, new_code):
        with self.operation_code_lock:
            self.operation_code = new_code

    def undistortion(self, distorted_plot, path):
        # 캘리브레이션 데이터 로드
        try:
            with open(path, 'rb') as f:
                calibration_data = pickle.load(f)
                camera_matrix = calibration_data["camera_matrix"]
                dist_coeffs = calibration_data["dist_coeffs"]
        except Exception as e:
            print("Calibration file error:", e)
        
        # 왜곡 보정
        plot = cv2.undistort(distorted_plot, camera_matrix, dist_coeffs)
        return plot


    def process_frames(self):
        while self.running:


            try:
                distorted_front_frame = self.front_frame_queue.get()
                distorted_rear_frame = self.rear_frame_queue.get()
            except queue.Empty:
                # 큐가 비어 있으면 다음 루프로
                time.sleep(0.01)
                continue
            
            # 최신 프레임을 유지하기 위해 큐를 비움
            while not self.front_frame_queue.empty():
                distorted_front_frame = self.front_frame_queue.get()
            while not self.rear_frame_queue.empty():
                distorted_rear_frame = self.rear_frame_queue.get()
            
            front_plot = cv2.flip(distorted_front_frame, 1)
            rear_plot = cv2.flip(distorted_rear_frame, 1)
                
            front_plot = self.undistortion(front_plot, FRONT_LANZ_CAL_PATH)
            rear_plot = self.undistortion(rear_plot, REAR_LANZ_CAL_PATH)
            
            # operation_code 읽을 때 락 사용
            with self.operation_code_lock:
                current_operation = self.operation_code
            if current_operation == 1:
                # 전면 카메라 이미지 예측
                front_results = self.model.predict(source=front_plot, classes=[0, 13, 15, 16, 28, 57], verbose=False)
                front_plot = front_results[0].plot()
                # 후면 카메라 이미지 예측 (필요 시 추가)
                rear_results = self.model.predict(source=rear_plot, classes=[0, 13, 15, 16, 28, 57], verbose=False)
                rear_plot = rear_results[0].plot()
            elif current_operation == 3:  # Following mode
                front_plot, sub_mode, diff_x, diff_y, body_size = self.body_follower.run(front_plot)
                self.reset_send_data()
                self.send_data["operating_code"] = current_operation
                self.send_data["camera"] = 1
                self.send_data["following"]["sub_mode"] = sub_mode
                self.send_data["following"]["diff_x"] = diff_x
                self.send_data["following"]["diff_y"] = diff_y
                self.send_data["following"]["body_size"] = body_size
                
            else:
                pass  # 다른 operation_code 처리
            # 디스플레이 큐에 넣기
            if self.front_display_queue.full():
                self.front_display_queue.get()
            self.front_display_queue.put(front_plot)
            if self.rear_display_queue.full():
                self.rear_display_queue.get()
            self.rear_display_queue.put(rear_plot)


            self.send_msg_queue.put(self.send_data)
            

    def reset_send_data(self):
        self.send_data = {
            "operating_code": 0,
            "camera": 1,             # 1: front 2: rear
            "following":{
                "sub_mode": 0,        # 0: stop & searching 1: too far 2: move to left 3: move to right 4: following 
                "diff_x": 9999,
                "diff_y": 9999,
                "body_size": 0,
            },
            "obstacle": {
                "detected": 0,
                "class": 0,         # 추후 정의 필요
                "conf": 0.0,
                "bbox_x": 0,
                "bbox_y": 0,
                "bbox_width": 0,
                "bbox_height": 0, 
            },
            "face": {
                "conf" : 0.0,
            }
        }

    def stop(self):
        self.running = False
        self.thread.join()

class FrameDisplayer:
    def __init__(self, front_display_queue, rear_display_queue):
        self.front_display_queue = front_display_queue
        self.rear_display_queue = rear_display_queue
        self.running = True

        # 스레드 시작
        self.thread = threading.Thread(target=self.display_frames, daemon=True)
        self.thread.start()

    def display_frames(self):
        while self.running:
 
            if not self.front_display_queue.empty():
                display_front_plot = self.front_display_queue.get()
                if display_front_plot is not None and display_front_plot.size > 0:
                    cv2.imshow("Front Camera", display_front_plot)
            if not self.rear_display_queue.empty():
                display_rear_plot = self.rear_display_queue.get()
                if display_rear_plot is not None and display_rear_plot.size > 0:
                    cv2.imshow("Rear Camera", display_rear_plot)

            # GUI 이벤트 처리
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        cv2.destroyAllWindows()

    def stop(self):
        self.running = False
        self.thread.join()

class ClientHandler:
    def __init__(self, client_socket, client_address, frame_processor, send_msg_queue):
        self.client_socket = client_socket
        self.client_address = client_address
        self.frame_processor = frame_processor

        self.send_msg_queue = send_msg_queue

        self.running = True

        # 송신스레드 시작
        self.thread = threading.Thread(target=self.handle_client_socket, daemon=True)
        self.thread.start()
        # 송신 스레드 시작
        self.thread_send = threading.Thread(target=self.send_to_client, daemon=True)
        self.thread_send.start()

    def handle_client_socket(self):
        logger.info(f"{self.client_address} connected!")
        try:
            while self.running:
                data = self.client_socket.recv(1024).decode('utf-8')
                if not data:
                    logger.info(f"({self.client_address[0]}) disconnected!")
                    break
                try:
                    json_data = json.loads(data)
                    new_operation_code = json_data.get("operating_code", self.frame_processor.operation_code)
                    self.frame_processor.update_operation_code(new_operation_code)
                except json.JSONDecodeError:
                    logger.warning("Received data is not valid JSON")
                except Exception as e:
                    logger.error(f"({self.client_address[0]}) : error! {e}")
                    break
        except Exception as e:
            logger.error(f"({self.client_address[0]}) : error! {e}")
        finally:
            self.client_socket.close()
            logger.info(f"Connection with {self.client_address} closed.")

    def send_to_client(self):
        """프레임 처리 결과를 주기적으로 클라이언트에 전송합니다."""
        while self.running: 
            if not self.send_msg_queue.empty():
                data = self.send_msg_queue.get()
                try:
                    # FrameProcessor에서 최신 데이터를 가져와서 전송
                    json_data = json.dumps(data)
                    self.client_socket.sendall(json_data.encode('utf-8'))
                    time.sleep(0.1)  
                except Exception as e:
                    logger.error(f"Error sending data to {self.client_address}: {e}")
                    break
            else:
                time.sleep(0.1)

    def stop(self):
        self.running = False
        self.thread.join()

class Server:
    def __init__(self, host, port, frame_processor, send_msg_queue):
        self.host = host
        self.port = port
        self.frame_processor = frame_processor
        self.send_msg_queue = send_msg_queue
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)  # 최대 5개 연결 대기
        self.running = True
        self.client_handlers = []

        # 스레드 시작
        self.thread = threading.Thread(target=self.accept_clients, daemon=True)
        self.thread.start()

    def accept_clients(self):
        logger.info("Server started and listening for connections.")
        while self.running:
            try:
                client_socket, client_address = self.server_socket.accept()
                handler = ClientHandler(client_socket, client_address, self.frame_processor, self.send_msg_queue)
                self.client_handlers.append(handler)
            except Exception as e:
                logger.error(f"Error accepting clients: {e}")
                break

    def stop(self):
        self.running = False
        self.server_socket.close()
        for handler in self.client_handlers:
            handler.stop()
        self.thread.join()

def main():
    # 큐 초기화
    front_frame_queue = queue.Queue(maxsize=10)
    rear_frame_queue = queue.Queue(maxsize=10)
    front_display_queue = queue.Queue(maxsize=10)
    rear_display_queue = queue.Queue(maxsize=10)
    send_msg_queue = queue.Queue(maxsize=10)

    # FrameReceiver 시작
    front_receiver = FrameReceiver(UDP_PORT2, front_frame_queue)
    front_receiver_thread = threading.Thread(target=front_receiver.start_receiving, daemon=True)
    front_receiver_thread.start()

    rear_receiver = FrameReceiver(UDP_PORT1, rear_frame_queue)
    rear_receiver_thread = threading.Thread(target=rear_receiver.start_receiving, daemon=True)
    rear_receiver_thread.start()

    # FrameProcessor 및 FrameDisplayer 인스턴스 생성
    frame_processor = FrameProcessor(front_frame_queue, rear_frame_queue, front_display_queue, rear_display_queue, send_msg_queue)
    frame_displayer = FrameDisplayer(front_display_queue, rear_display_queue)

    # 서버 인스턴스 생성
    server = Server(HOST, PORT, frame_processor, send_msg_queue)

    # 메인 스레드에서 종료 신호 대기
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        server.stop()
        frame_processor.stop()
        frame_displayer.stop()

if __name__ == "__main__":
    main()

