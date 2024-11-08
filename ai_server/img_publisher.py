import socket
import cv2
import struct
import threading
import time

# 서버 IP 및 포트 설정
MULTICAST_IP = '224.1.1.1'
UDP_PORT1 = 9997  # 첫 번째 카메라용 포트
UDP_PORT2 = 9996  # 두 번째 카메라용 포트
MAX_DGRAM = 1300  # 패킷 크기를 MTU 이하로 설정하여 단편화 방지

# 로봇 번호 및 응답 코드 설정
bg_num = 1     # (baby goose number)
br_code = 10   # (baby result code)

def send_frame(cap, udp_socket, udp_port, frame_id):
    """
    프레임을 바이너리 데이터로 UDP로 전송하는 함수
    """
    ret, frame = cap.read()
    if not ret:
        print(f"카메라에서 프레임을 읽을 수 없습니다.")
        return None  # 프레임을 읽을 수 없으면 None 반환

    frame_id = (frame_id + 1) % 65536  # 0-65535 사이에서 순환

    # 해상도와 압축률 조절
    frame = cv2.resize(frame, (500, 375))  # 해상도 재조정
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]  # 압축하기
    result, frame_jpg = cv2.imencode('.jpg', frame, encode_param)
    data = frame_jpg.tobytes()

    # 데이터 분할
    chunk_size = MAX_DGRAM - 8  # 헤더 크기 고려 (8바이트)
    size = len(data)
    total_chunks = (size + chunk_size - 1) // chunk_size

    header_format = '=HBBHBB'  # frame_id(2바이트), total_chunks(1바이트), seq_num(1바이트), data_len(2바이트), bg_num(1바이트), br_code(1바이트)
    for seq_num in range(total_chunks):
        start = seq_num * chunk_size
        end = min(start + chunk_size, size)
        chunk = data[start:end]

        # 헤더 구성
        header = struct.pack(header_format, frame_id, total_chunks, seq_num, len(chunk), bg_num, br_code)
        udp_socket.sendto(header + chunk, (MULTICAST_IP, udp_port))

    # 전송 속도 조절
    time.sleep(0.05)  # 약 20fps

    print(f"포트 {udp_port}로 프레임 {frame_id} 전송 완료")
    return frame_id

def camera_thread(cap_index, udp_port):
    cap = cv2.VideoCapture(cap_index)
    if not cap.isOpened():
        print(f"카메라 {cap_index}을 열 수 없습니다.")
        return
        
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # multicast socket setting
    ttl = struct.pack('b', 1)  # Time-to-live 설정
    udp_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

    frame_id = 0
    while True:
        frame_id = send_frame(cap, udp_socket, udp_port, frame_id)
        if frame_id is None:
            break  # 프레임을 읽을 수 없으면 루프 탈출

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    udp_socket.close()

# 스레드 생성
thread1 = threading.Thread(target=camera_thread, args=(0, UDP_PORT1))
thread2 = threading.Thread(target=camera_thread, args=(2, UDP_PORT2))

# 스레드 시작
thread1.start()
thread2.start()

thread1.join()
thread2.join()

