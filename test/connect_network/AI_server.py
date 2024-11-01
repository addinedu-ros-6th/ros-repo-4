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

model = YOLO("yolov8s.pt")

class FrameReceiver:
    def __init__(self, udp_port):
        # UDP 소켓 설정
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(('0.0.0.0', udp_port))
        self.frame_buffer = {}
        self.lock = threading.Lock()
        self.frame_queue = queue.Queue()  # 프레임을 전달할 큐
        self.udp_port = udp_port  # 디버깅을 위한 포트 정보 저장

    def receive_data(self):
        while True:
            try:
                segment, addr = self.udp_socket.recvfrom(MAX_DGRAM)
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

def main():
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

            if not receiver2.frame_queue.empty():
                frame2 = receiver2.frame_queue.get_nowait()
                results2 = model.predict(source=frame2, classes=[0], verbose=False)
                plots2 = results2[0].plot()
                cv2.imshow("Camera 2", plots2)

            if not receiver3.frame_queue.empty():
                frame3 = receiver3.frame_queue.get_nowait()
                results3 = model.predict(source=frame3, classes=[0], verbose=False)
                plots3 = results3[0].plot()
                cv2.imshow("Camera 3", plots3)

            if not receiver4.frame_queue.empty():
                frame4 = receiver4.frame_queue.get_nowait()
                results4 = model.predict(source=frame4, classes=[0], verbose=False)
                plots4 = results4[0].plot()
                cv2.imshow("Camera 4", plots4)

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

if __name__ == "__main__":
    main()
