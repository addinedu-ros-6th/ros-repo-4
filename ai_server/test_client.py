import socket
import json
import time

# 서버 설정
HOST = '192.168.0.45'  # 서버 IP 주소
PORT = 8888         # 서버 포트 번호

# JSON 데이터 생성
data = {
    "operating_code": 3
}

# TCP 클라이언트 소켓 생성
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

try:
    while True:
        # JSON 데이터를 문자열로 변환 및 인코딩 후 전송
        json_data = json.dumps(data)
        client_socket.sendall(json_data.encode('utf-8'))
        print("send!!")
        
        # 0.05초 대기
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Terminated by user.")

finally:
    client_socket.close()
    print("Connection closed.")