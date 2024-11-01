## DESKTOP(server)

udp로 영상을 받는 파이썬 파일과 tcp/ip로 데스크탑과 라즈베리파이 쌍방향 통신 실행

```
python admin.py
```
## 데모 프로그램 설명

  - UDP : 라즈베리파이에서 영상을 송출 -> AI_server.py(DESKTOP)에서 받은 영상으로 yolo 추론 수행 후 opencv로 영상 도시
  - TCPIP : DESKTOP에서 각 라즈베리 파이에 숫자 100을 보냄 -> 미니봇은 받은 숫자를 16진수로 변환 후 터미널에 출력 ->  받은 숫자에 +1 16진수 값을 DESKTOP에 전송(1번 봇은 2초마다, 2번 봇은 3초마다) -> DESKTOP은 받은 숫자를 10진수로 변환후 출력 -> DESKTOP에서 받은 숫자를 다시 라즈베리파이에 전송 -> 무한 반복

### client(라즈베리파이)

1. ssh로 로봇 접속
2. googeesebot (activate해주기)
3. 터미널 분리
4. 1번 터미널에서 ros2 launch opencv_tutorials camera.launch.py (UDP로 라즈베리파이 -> DESKTOP에 영상 송신)
5. 2번 터미널에서 python3 tcp_ip_client.py 실행
