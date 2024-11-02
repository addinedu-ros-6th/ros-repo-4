# client.py
import socket
import struct
import time

GON_ADDIN_5G = '192.168.0.45'
GON_HOME = '192.168.1.16'

SERVER_HOST =  GON_HOME # 서버 IP 주소
SERVER_PORT = 8888  # 서버 포트 번호

def connect_to_server():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((SERVER_HOST, SERVER_PORT))
    print("서버에 연결됨.")

    try:
        while True:
            # 서버로부터 기존 UDP 전송했던 port (unsigned short : 2bytes) 와 
            # YOLO person 신뢰도 값 (float : 4 bytes)을 전송 받응 
            data = client_socket.recv(6)  # 6바이트 정수
            if data:
                (port, conf) = struct.unpack('!Hf', data)
                print(f"서버로부터 받은 port & conf: ({port}, {conf})")

            # conf 값이 0.85 이상이면 1, 아니면 0을 보냄
            number = 1 if conf >= 0.85 else 0 
                            
            packed_number = struct.pack('!I', number)
            client_socket.sendall(packed_number)
            print(f"클라이언트에서 {number}를 전송했습니다.")
            
    except KeyboardInterrupt:
        print("연결 종료됨.")
    finally:
        client_socket.close()

if __name__ == "__main__":
    connect_to_server()
