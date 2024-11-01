## DESKTOP(server)

실행

```
python admin.py
```

### 프로그램 실행


# 프로그램 설명


### client(라즈베리파이)

1. ssh로 로봇 접속
2. googeesebot (activate해주기)
3. 터미널 분리
4. 1번 터미널에서 ros2 launch opencv_tutorials camera.launch.py (UDP로 라즈베리파이 -> DESKTOP에 영상 송신)
5. 2번 터미널에서 python3 tcp_ip_client.py 실행
