import socket
import struct
import cv2
import numpy as np

# 멀티캐스트 설정
MULTICAST_IP = '224.1.1.1'
UDP_PORT = 9996
MAX_DGRAM = 1300  # 송신에서 설정한 패킷 크기

# 로봇 번호 및 응답 코드
bg_num = 1     # (baby goose number)
br_code = 10   # (baby result code)

def receive_frames():
    """
    멀티캐스트 UDP로 수신한 데이터를 조합하여 화면에 표시하는 함수
    """
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind((MULTICAST_IP, UDP_PORT))

    # 멀티캐스트 그룹 가입
    mreq = struct.pack("4sl", socket.inet_aton(MULTICAST_IP), socket.INADDR_ANY)
    udp_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    buffer = {}
    last_frame_id = -1

    while True:
        try:
            # 데이터 수신
            data, addr = udp_socket.recvfrom(MAX_DGRAM)
            
            # 헤더 파싱
            header_format = '=HBBHBB'  # frame_id(2바이트), total_chunks(1바이트), seq_num(1바이트), data_len(2바이트), bg_num(1바이트), br_code(1바이트)
            header_size = struct.calcsize(header_format)
            header = data[:header_size]
            frame_id, total_chunks, seq_num, data_len, recv_bg_num, recv_br_code = struct.unpack(header_format, header)
            chunk_data = data[header_size:]

            # 로봇 번호와 응답 코드 검증
            if recv_bg_num != bg_num or recv_br_code != br_code:
                continue

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
