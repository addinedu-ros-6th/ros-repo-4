import socket
import struct
import cv2
import numpy as np

# UDP 수신 설정
UDP_IP = '192.168.200.191'  # 수신할 IP (localhost)
UDP_PORT = 9996       # 수신할 포트
MAX_DGRAM = 1300      # 송신에서 설정한 패킷 크기

def receive_frames():
    """
    UDP로 수신한 데이터를 조합하여 화면에 표시하는 함수
    """
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((UDP_IP, UDP_PORT))

    buffer = {}
    last_frame_id = -1

    while True:
        try:
            # 데이터 수신
            data, addr = udp_socket.recvfrom(MAX_DGRAM)
            
            # 헤더 파싱
            header_format = '=HBBH'  # frame_id(2바이트), total_chunks(1바이트), seq_num(1바이트), data_len(2바이트)
            header_size = struct.calcsize(header_format)
            header = data[:header_size]
            frame_id, total_chunks, seq_num, data_len = struct.unpack(header_format, header)
            chunk_data = data[header_size:]

            # 새로운 프레임이면 버퍼 초기화
            if frame_id != last_frame_id:
                buffer = {}
                last_frame_id = frame_id

            # 버퍼에 조각 저장
            buffer[seq_num] = chunk_data

            # 모든 조각을 수신했는지 확인
            if len(buffer) == total_chunks:
                # 프레임 데이터 조합
                frame_data = b"".join(buffer[seq] for seq in sorted(buffer))
                buffer = {}  # 버퍼 초기화

                # 프레임 디코딩 및 표시
                frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    cv2.imshow("Received Frame", frame)

                    # 'q' 키를 누르면 종료
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

        except Exception as e:
            print(f"에러 발생: {e}")
            break

    udp_socket.close()
    cv2.destroyAllWindows()

# 실행
receive_frames()
