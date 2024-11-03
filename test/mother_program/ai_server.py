import threading
import socket
import struct
import numpy as np
import cv2
import time
import queue
from ultralytics import YOLO


MAX_DGRAM = 65507  # UDP의 최대 패킷 크기
UDP_PORT1 = 9996   # 첫 번째 내장 카메라용 포트
UDP_PORT2 = 9997   # 두 번째 카메라용 포트
UDP_PORT3 = 9998   # 세 번째 내장 카메라용 포트
UDP_PORT4 = 9999   # 네 번째 카메라용 포트

GON_ADDIN_5G = '192.168.0.45'
GON_HOME = '192.168.1.16'
MULTICAST_IP = '224.1.1.1'

model = YOLO("yolov8s.pt")

class FrameReceiver:
    def __init__(self, udp_port):
        self.port = udp_port

        # UDP 소켓 설정
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((MULTICAST_IP, udp_port))
        group = socket.inet_aton(MULTICAST_IP)
        mreq = struct.pack('4sL', group, socket.INADDR_ANY)
        self.udp_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        self.frame_buffer = {}
        self.lock = threading.Lock()
        self.frame_queue = queue.Queue()  # 프레임을 전달할 큐
        self.udp_port = udp_port  # 디버깅을 위한 포트 정보 저장

    def receive_data(self):
        while True:
            try:
                segment, addr = self.udp_socket.recvfrom(MAX_DGRAM)
                
                self.client_ip = addr[0]
                
                if len(segment) < 8:
                    continue  # 잘못된 패킷 무시

                # 헤더 파싱
                frame_id, total_chunks, seq_num, data_len, bg_num, br_code = struct.unpack('=HBBHBB', segment[:8])
                data = segment[8:]

                key = (addr, frame_id)
                with self.lock:
                    if key not in self.frame_buffer:
                        self.frame_buffer[key] = {
                            'total_chunks': total_chunks,
                            'chunks': [None] * total_chunks,
                            'received_chunks': 0,
                            'timestamp': time.time()
                        }

                    frame_info = self.frame_buffer[key]
                    if seq_num < total_chunks and frame_info['chunks'][seq_num] is None:
                        frame_info['chunks'][seq_num] = data
                        frame_info['received_chunks'] += 1

                    # 모든 청크를 수신한 경우 프레임 조립
                    if frame_info['received_chunks'] == frame_info['total_chunks']:
                        # 프레임 완성
                        frame_data = b''.join(frame_info['chunks'])
                        frame = np.frombuffer(frame_data, dtype=np.uint8)
                        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
                        if frame is not None:
                            self.frame_queue.put(frame)  # 큐에 프레임 추가
                        else:
                            print(f"포트 {self.udp_port}: 프레임 디코딩 실패")
                        del self.frame_buffer[key]
                    else:
                        # 일정 시간 내에 모든 청크를 받지 못하면 해당 프레임 삭제
                        if time.time() - frame_info['timestamp'] > 1:  # 타임아웃 시간 1초로 조절
                            print(f"포트 {self.udp_port}: 프레임 {frame_id} 수신 시간 초과로 삭제합니다.")
                            del self.frame_buffer[key]

            except Exception as e:
                print(f"포트 {self.udp_port}: 데이터 수신 중 오류 발생: {e}")

    def start_receiving(self):
        receive_thread = threading.Thread(target=self.receive_data, daemon=True)
        receive_thread.start()


class AiServer:
    def __init__(self):   # UDP 소켓 설정
        self.input_queue = queue.Queue()
        self.output_queue = queue.Queue()

    def pipe_connector(self, ai_pipe):
        while True:
            # 먼저 파이프를 체크합니다.
            if ai_pipe.poll():
                recv_data = ai_pipe.recv()
                self.input_queue.put(recv_data)
            
            # gui_queue를 비블로킹으로 체크합니다.
            try:
                send_data = self.output_queue.get_nowait()
                
                # print(f"send data : {send_data}")
                if send_data is None:
                    break
                ai_pipe.send(send_data)
                self.output_queue.task_done()
            except queue.Empty:
                pass  # 큐가 비어 있으면 넘어갑니다.

            # 너무 빠른 루프를 방지하기 위해 약간 대기합니다.
            time.sleep(0.01)  # 10ms 대기
 

    def run(self, ai_pipe):
        comm_thread = threading.Thread(target=self.pipe_connector, args=(ai_pipe,), daemon=True)
        comm_thread.start()

        # 네 개의 프레임 수신기 생성
        receiver1 = FrameReceiver(UDP_PORT1)
        receiver2 = FrameReceiver(UDP_PORT2)
        receiver3 = FrameReceiver(UDP_PORT3)
        receiver4 = FrameReceiver(UDP_PORT4)

        # 수신 시작
        receiver1.start_receiving()
        receiver2.start_receiving()
        receiver3.start_receiving()
        receiver4.start_receiving()

        # 메인 스레드에서 프레임 표시
        while True:
            try:
                if not receiver1.frame_queue.empty():
                    frame1 = receiver1.frame_queue.get_nowait()
                    results1 = model.predict(source=frame1, classes=[0], verbose=False)
                    plots1 = results1[0].plot()
                    cv2.imshow("Camera 1", plots1)

                    # 검출 신뢰도 값 기본 설정 
                    detected_confidnece1 = None

                    for result in results1:
                        if result.boxes is not None:
                            for box in result.boxes:
                                if box.cls == 0: # 사람 클래스 (class 0)
                                    detected_confidnece1 = box.conf.item()

                                    # client ip, connected port, YOLO 검출 값 전달
                                    message = (receiver1.client_ip, receiver1.port, detected_confidnece1)
                                    self.output_queue.put(message)
                                    break

                if not receiver2.frame_queue.empty():
                    frame2 = receiver2.frame_queue.get_nowait()
                    results2 = model.predict(source=frame2, classes=[0], verbose=False)
                    plots2 = results2[0].plot()
                    cv2.imshow("Camera 2", plots2)

                    # 검출 신뢰도 값 기본 설정 
                    detected_confidnece2 = None

                    for result in results2:
                        if result.boxes is not None:
                            for box in result.boxes:
                                if box.cls == 0: # 사람 클래스 (class 0)
                                    detected_confidnece2 = box.conf.item()

                                    # client ip, connected port, YOLO 검출 값 전달
                                    message = (receiver2.client_ip, receiver2.port, detected_confidnece2)
                                    self.output_queue.put(message)
                                    break

                if not receiver3.frame_queue.empty():
                    frame3 = receiver3.frame_queue.get_nowait()
                    results3 = model.predict(source=frame3, classes=[0], verbose=False)
                    plots3 = results3[0].plot()
                    cv2.imshow("Camera 3", plots3)

                    # 검출 신뢰도 값 기본 설정 
                    detected_confidnece3 = None

                    for result in results3:
                        if result.boxes is not None:
                            for box in result.boxes:
                                if box.cls == 0: # 사람 클래스 (class 0)
                                    detected_confidnece3 = box.conf.item()

                                    # client ip, connected port, YOLO 검출 값 전달
                                    message = (receiver3.client_ip, receiver3.port, detected_confidnece3)
                                    self.output_queue.put(message)
                                    break

                if not receiver4.frame_queue.empty():
                    frame4 = receiver4.frame_queue.get_nowait()
                    results4 = model.predict(source=frame4, classes=[0], verbose=False)
                    plots4 = results4[0].plot()
                    cv2.imshow("Camera 4", plots4)

                    # 검출 신뢰도 값 기본 설정 
                    detected_confidnece1 = None

                    for result in results4:
                        if result.boxes is not None:
                            for box in result.boxes:
                                if box.cls == 0: # 사람 클래스 (class 0)
                                    detected_confidnece4 = box.conf.item()

                                    # client ip, connected port, YOLO 검출 값 전달
                                    message = (receiver4.client_ip, receiver4.port, detected_confidnece4)
                                    self.output_queue.put(message)
                                    break

            except queue.Empty:
                pass

            # 키 입력 처리
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # 종료 처리
        receiver1.udp_socket.close()
        receiver2.udp_socket.close()
        receiver3.udp_socket.close()
        receiver4.udp_socket.close()
        cv2.destroyAllWindows()