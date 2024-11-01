# server.py
import socket
import struct
import threading

HOST = '192.168.0.30'  # 데스크탑의 IP 주소
PORT = 8888  # 포트 번호

def handle_client(client_socket, client_address):
    print(f"{client_address}에서 연결됨.")
    try:
        # 초기 정수 값 설정
        number = 100  # 예시 정수
        packed_number = struct.pack('!I', number)
        
        while True:
            # 클라이언트에게 정수 전송
            client_socket.sendall(packed_number)
            print(f"{client_address}에게 {number}를 전송했습니다.")

            # 클라이언트로부터 응답 받기
            data = client_socket.recv(4)
            if not data:
                break

            # 클라이언트로부터 받은 데이터를 언패킹하여 10진수로 출력
            unpacked_number = struct.unpack('!I', data)[0]
            print(f"{client_address}로부터 받은 값(10진수): {unpacked_number}")

            # 다음 전송 값 설정
            number = unpacked_number

    except ConnectionResetError:
        print(f"{client_address} 연결이 끊어졌습니다.")
    finally:
        client_socket.close()
        print(f"{client_address} 연결 종료됨.")

def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 포트 재사용 옵션 추가
    server_socket.bind((HOST, PORT))
    server_socket.listen()
    print(f"서버가 {HOST}:{PORT}에서 실행 중...")

    while True:
        client_socket, client_address = server_socket.accept()
        client_handler = threading.Thread(target=handle_client, args=(client_socket, client_address))
        client_handler.start()

if __name__ == "__main__":
    start_server()
