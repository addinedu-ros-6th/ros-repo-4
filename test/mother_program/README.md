## 프로그램 구조도 
https://addros.atlassian.net/wiki/spaces/CC/pages/23691266/Mother+System

## DESKTOP(server)

udp로 영상을 받는 파이썬 파일과 tcp/ip로 데스크탑과 라즈베리파이 쌍방향 통신 실행

```
python admin.py
```
## 데모 프로그램 설명

  - UDP : 라즈베리파이에서 영상을 송출 <br>
  -> AI_process (ai_server.py)에서 받은 영상으로 yolo 추론 수행 후 opencv로 영상 도시<br>
  -> googeese bot IP, 연결된 server port, yolo 추론 confidence 결과를 queue 를 통해 pipe thread 로 전달<br>
  -> pipe 를 통해 control process(control_server.py) 로 data 전송<br>
  -> control process 에 TCP client queue 넣어 전달<br>
  -> TCP client thread 에서 처리 데이터를 받아 다시 googeesebot 에게 TCP/IP 통신을 전달<br>
  -> googesebot TCP client 에서 받아서 confidence 값이  0.85 이상이면 1 아니면 0 을 sever 에 응답 <br>
  -> server (control_process) 에서 client 응답 값 (0 or 1) 확인

### client(라즈베리파이)

1. ssh로 로봇 접속
2. googeesebot img_publiser.py sever ip 확인 만약 수정했다면 colcon build --pakcages-select opencv_tutorials 필요
3. 해당 위치 for_googeesebot 폴더 내  tcp_ip_client.py 파일에서 역시 server ip 를 현재 조건에 맞게 설정 후  googeesebot 내 해당 파일 교체
4. 1번 터미널에서 ros2 launch opencv_tutorials camera.launch.py (UDP로 라즈베리파이 -> DESKTOP에 영상 송신)
5. 2번 터미널에서 python3 tcp_ip_client.py 실행
