import threading
import queue
import socket
import struct
import threading
import time

GON_ADDIN_5G = '192.168.0.45'
GON_HOME = '192.168.1.16'

HOST = GON_HOME  # 데스크탑의 IP 주소
PORT = 8888  # 포트 번호


class ControlServer:
    def __init__(self):   # UDP 소켓 설정
        # self.input_queue = queue.Queue()
        self.output_queue = queue.Queue()
        self.client_ip_queue_dict = {}

    def handle_client(self, client_socket, client_address):
        ip = client_address[0]
        print(f"{client_address}에서 연결됨.")

        try:
            while True:
                try:
                    (recv_port, recv_conf) = self.client_ip_queue_dict[ip].get(block=False)
                    print(f"ai_result : {(recv_port, recv_conf)}") 
                    packed_data = struct.pack('!Hf', recv_port, recv_conf) # H : unsigned short (2bytes), f float (4bytes)

                    client_socket.sendall(packed_data)  
                    print(f"{client_address}에게 {(recv_port, recv_conf)}를 전송했습니다.")
                    
                    # 클라이언트로부터 응답 받기
                    data = client_socket.recv(4)
                    if not data:
                        continue

                    # 클라이언트로부터 받은 데이터를 언패킹하여 10진수로 출력
                    unpacked_number = struct.unpack('!I', data)[0]
                    print(f"{client_address}로부터 받은 값(10진수): {unpacked_number}")

                except queue.Empty:
                    # 큐가 비어있을 때는 일정 시간 대기 후 다시 시도
                    time.sleep(0.01)



        except ConnectionResetError:
            print(f"{client_address} 연결이 끊어졌습니다.")
        finally:
            client_socket.close()
            print(f"{client_address} 연결 종료됨.")

    def pipe_connector(self, control_pipe):
        while True:
            # 먼저 파이프를 체크합니다.
            if control_pipe.poll():
                (recv_ip, recv_port, recv_conf) = control_pipe.recv() # (ip, port, result)
                # print(f"recv_data : {(recv_ip, recv_port, recv_conf)}")

                # AI process 로 부터 받은 data 에 저장된 UDP IP 와 현재 연결된 TCP client 를 비교
                # 동일 IP 가 존재 하는 경우, 해당 TCP handler Queue 에 전달된 메세지를 담음 
                if recv_ip in self.client_ip_queue_dict.keys():
                    self.client_ip_queue_dict[recv_ip].put((recv_port, recv_conf)) # UDP port 와 딥러닝 결과 값만 보냄
                else:
                    print(f"{recv_ip} : TPC_IP connection is not exist")
                
                # self.input_queue.put(recv_data)
            
            # gui_queue를 비블로킹으로 체크합니다.
            try:
                send_data = self.output_queue.get_nowait()
                if send_data is None:
                    break
                control_pipe.send(send_data)
                self.output_queue.task_done()
            except queue.Empty:
                pass  # 큐가 비어 있으면 넘어갑니다.

            # 너무 빠른 루프를 방지하기 위해 약간 대기합니다.
            time.sleep(0.01)  # 10ms 대기

    def run(self, control_pipe):
        comm_thread = threading.Thread(target=self.pipe_connector, args=(control_pipe,), daemon=True)
        comm_thread.start()

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 포트 재사용 옵션 추가
        server_socket.bind((HOST, PORT))
        server_socket.listen()
        print(f"서버가 {HOST}:{PORT}에서 실행 중...")

        while True:
            client_socket, client_address = server_socket.accept()

            # 각 TCP client IP 에 대한 Queue 를 생성
            self.client_ip_queue_dict[client_address[0]] = queue.Queue()

            client_handler = threading.Thread(target=self.handle_client, args=(client_socket, client_address))
            client_handler.start()
