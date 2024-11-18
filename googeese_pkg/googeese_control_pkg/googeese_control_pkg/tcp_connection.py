import threading
import socket
import json
import queue
import time


class TCPConnect(object):
    """Server 와 데이터 송신/수신"""
    def __init__(self, host, port, send_q, recv_q, interval):
        self.host = host
        self.port = port
        self.send_q = send_q
        self.recv_q = recv_q
        self.interval = interval

        # server 통신 TCP 클라이언트 소켓 생성
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        try:
            client_socket.connect((host, port))
            print(f"Connected to server at {host}:{port}")
        except Exception as e:
            print(f"Failed to connect to server: {e}")
            return
        
        # server 에 mode 번호 전달할 송신 thread 생성 및 시작
        self.send_thread = threading.Thread(
            target=self.send_server,
            args=(client_socket, ),
            daemon=True
            )
        self.send_thread.start()

        # server 에 딥러닝 결과를 받을 수신 thread 생성 및 시작
        self.recv_thread = threading.Thread(
            target=self.recv_server,
            args=(client_socket, ),
            daemon=True
            )
        self.recv_thread.start()

    def send_server(self, client_socket):
        """서버에 데이터를 큐로 받아서 지속적으로 전송하는 함수"""
        try:
            while True:
                if not self.send_q.empty():
                    data = self.send_q.get()

                    # JSON 데이터를 문자열로 변환 및 인코딩 후 전송
                    json_data = json.dumps(data)
                    json_data += '\n'
                    client_socket.sendall(json_data.encode('utf-8'))
                else:
                    time.sleep(self.interval)
        except Exception as e:
            print(f"Send thread error: {e}")
    
    def recv_server(self, client_socket):
        """서버에 데이터를 지속적으로 응답을 받아 큐로 전달하는 함수"""
        buffer = ""
        try:
            while True:
                # 서버로부터 데이터 수신 (최대 1024 바이트)
                response = client_socket.recv(1024).decode('utf-8')
                if not response:
                    # 서버가 연결을 종료한 경우
                    print("Server closed the connection.")
                    break
                buffer += response
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip() == "":
                        continue  # 빈 줄은 무시
                    try:
                        # 수신한 데이터를 디코딩하고 JSON으로 변환
                        json_response = json.loads(line)
                        self.recv_q.put(json_response)
                        time.sleep(self.interval)
                    except json.JSONDecodeError:
                        print("Received non-JSON data:", response.decode('utf-8'))
        except Exception as e:
            print(f"Receive thread error: {e}")

