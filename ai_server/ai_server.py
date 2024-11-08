import socket
import threading
import sys
import json 
import cv2
import time
import queue
from ultralytics import YOLO
from body_follower import *
from udp_connection import *
from aruco_identification import *

# 서버 설정
HOST = '127.0.0.1'  # 서버 IP 주소 (localhost)
PORT = 8888         # 사용할 포트 번호

SEND_JSON = {

}

class AiTask():
    def __init__(self, front_frame_queue, rear_frame_queue, front_display_queue, rear_display_queue):
        self.model = YOLO("yolov8s.pt")

        self.front_frame_queue = front_frame_queue
        self.rear_frame_queue = rear_frame_queue

        self.front_plot = None
        self.rear_plot = None

        # Display queues for passing frames to the main thread
        self.front_display_queue = front_display_queue
        self.rear_display_queue = rear_display_queue

        self.operation_code = 0  

        self.body_follower = BodyFollower()

    
    def run(self, client_socket, client_address):
        print(f"{client_address} connected!")

        while True:
            data = client_socket.recv(1024).decode('utf-8')
            try:
                if not data:
                    print(f"({client_address[0]}) disconnected!")
                    break
                else:
                    try:
                        json_data = json.loads(data)
                        self.operation_code = json_data["operating_code"]

                    except json.JSONDecodeError:
                        print("Received data is not valid JSON")
            except Exception as e:
                print(f"({client_address[0]}) : error! {e}")
                break  

            print(f"{self.front_frame_queue.empty()}")
            if not self.front_frame_queue.empty():
                # 밀려있는 frame 을 소모시켜서 영상 지연을 줄임
                while not self.front_frame_queue.empty():
                    self.front_plot = cv2.flip(self.front_frame_queue.get(), 1)
                    self.rear_plot = cv2.flip(self.rear_frame_queue.get(), 1)
                
                if self.operation_code == 1:
                    # 전면 카메라 이미지 예측  0:사람, 13:벤치, 15: 고양이, 16:개, 28:suitcase, 57:의자
                    front_results = self.model.predict(source=self.front_plot, classes=[0, 13, 15, 16, 28, 57], verbose=False)
                    # Prepare the image with detections for display
                    self.front_plot = front_results[0].plot()

                    # 전면 카메라 첫 번째 탐지된 객체의 정보를 사용
                    if front_results and len(front_results) > 0 and len(front_results[0].boxes) > 0:
                        pass

                elif self.operation_code == 3: # Following mode
                    self.front_plot, sub_mode, diff_x, diff_y, body_size = self.body_follower.run(self.front_plot)
                else:
                    pass

                # Put the image into the display queue
                if self.front_display_queue.full():
                    self.front_display_queue.get()
                if self.front_plot is not None:
                    self.front_display_queue.put(self.front_plot)
                
                # Put the image into the display queue
                if self.rear_display_queue.full():
                    self.rear_display_queue.get()
                if self.rear_plot is not None:
                    self.rear_display_queue.put(self.rear_plot)

        client_socket.close()
        print(f"Connection with {client_address} closed.")


    def display_frames(self):
        while True:
            # Check if there are frames to display
            if not self.front_display_queue.empty():
                display_front_plot = self.front_display_queue.get()
                # 이미지가 유효한지 확인
                if display_front_plot is not None and display_front_plot.size > 0:
                    cv2.imshow("Front Camera", display_front_plot)
            if not self.rear_display_queue.empty():
                display_rear_plot = self.rear_display_queue.get()
                # 이미지가 유효한지 확인
                if display_rear_plot is not None and display_rear_plot.size > 0:
                    cv2.imshow("Rear Camera", display_rear_plot)

            # Wait for a key press for 1 ms to handle GUI events
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()


def accept_clients(server_socket):
    #"""클라이언트 연결을 수락하는 함수"""
    while True:
        client_socket, client_address = server_socket.accept()

        # AiTask 노드 생성
        ai_task = AiTask(front_frame_queue, rear_frame_queue, front_display_queue, rear_display_queue)

        threading.Thread(
            target=ai_task.run, 
            args=(client_socket, client_address)
            ).start()
        
        ai_task.display_frames()

if __name__ == "__main__":
    front_frame_queue = queue.Queue(maxsize=10)
    rear_frame_queue = queue.Queue(maxsize=10)

    # Queues for frames to be displayed
    front_display_queue = queue.Queue(maxsize=10)
    rear_display_queue = queue.Queue(maxsize=10)

    # FrameReceiver 시작
    front_receiver = FrameReceiver(UDP_PORT1, front_frame_queue)
    front_receiver_thread = threading.Thread(target=front_receiver.start_receiving, daemon=True)
    front_receiver_thread.start()

    rear_receiver = FrameReceiver(UDP_PORT2, rear_frame_queue)
    rear_receiver_thread = threading.Thread(target=rear_receiver.start_receiving, daemon=True)
    rear_receiver_thread.start()

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_address = (HOST, PORT)

    server_socket.bind(server_address)
    server_socket.listen(5)  # 최대 5개 연결 대기

    print("server started!")

    # 클라이언트 연결을 수락하는 스레드
    threading.Thread(target=accept_clients, args=(server_socket,)).start()

    
