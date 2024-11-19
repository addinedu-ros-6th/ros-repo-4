import socket
import threading
import json
import time

TIME_INTERVAL = 0.5

class TCPServer():
    def __init__(self, host, port, send_msg_q, recv_msg_q):
        self.running = True

        self.host = host
        self.port = port
        self.send_msg_q = send_msg_q
        self.recv_msg_q = recv_msg_q
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        self.socket.bind((host, port))
        self.socket.listen(10)  
        print(f"({host},{port}) Listening... ")


        self.accept_thread = threading.Thread(
            target=self.accept_clients,
            daemon=True
        )
        self.accept_thread.start()

    def accept_clients(self):
        while self.running:
            try:
                client_socket, _client_address = self.socket.accept()                
                print(f"Connected : {client_socket}")
                send_thread = threading.Thread(
                    target=self.send_client, 
                    args=(client_socket, ),
                    daemon=True
                    )
                send_thread.start()
                
                recv_thread = threading.Thread(
                    target=self.recv_client, 
                    args=(client_socket, ),
                    daemon=True
                    )
                recv_thread.start()

            except Exception as e:
                print(f"Error accepting clients: {e}")
                break

    def send_client(self, client_socket):
        try:
            while True:
                if not self.send_msg_q.empty():
                    data = self.send_msg_q.get()
                    json_data = json.dumps(data)
                    json_data += '\n'
                    client_socket.sendall(json_data.encode('utf-8'))
                else:
                    time.sleep(TIME_INTERVAL)
        except Exception as e:
            print(f"Send thread error: {e}")

    def recv_client(self, client_socket):
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
                        # print(json_response)
                        self.recv_msg_q.put(json_response)
                        time.sleep(TIME_INTERVAL)
                    except json.JSONDecodeError:
                        print("Received non-JSON data:", response.decode('utf-8'))
        except Exception as e:
            print(f"Receive thread error: {e}")

    def stop(self):
        self.running = False
        self.socket.close()

