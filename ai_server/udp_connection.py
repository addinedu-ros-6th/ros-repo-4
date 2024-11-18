import threading
import socket
import struct
import numpy as np
import cv2
import time
import queue

UDP_PORT1 = 9996   # 첫 번째 내장 카메라용 포트
UDP_PORT2 = 9997   # 두 번째 카메라용 포트
UDP_PORT3 = 9998   # 세 번째 내장 카메라용 포트
UDP_PORT4 = 9999   # 네 번째 카메라용 포트

MAX_DGRAM = 65507  # UDP의 최대 패킷 크기
MULTICAST_IP = '224.1.1.1'
# UDP_IP = '192.168.1.16'  

class FrameReceiver:
    def __init__(self, udp_port, frame_queue):
        self.port = udp_port

        # UDP 소켓 설정
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # multicast 사용 시
        self.udp_socket.bind((MULTICAST_IP, udp_port))
        group = socket.inet_aton(MULTICAST_IP)
        mreq = struct.pack('4sL', group, socket.INADDR_ANY)
        self.udp_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        # 일반 UDP 사용 시
        # self.udp_socket.bind((UDP_IP, udp_port))

        self.frame_buffer = {}
        self.lock = threading.Lock()
        self.frame_queue = frame_queue
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
                            # 큐가 가득 찼으면 오래된 Frame 제거
                            if self.frame_queue.full():
                                self.frame_queue.get()   # 오래된 프레임 삭제
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
