## 설치해야 할 모듈 


### 미니 봇에서 실행할 때 설정
spidev 설치
```
sudo chmod 660 /dev/gpiomem
sudo chown root:gpio /dev/gpiomem #라즈베리파이 GPIO설정
sudo usermod -aG spi $(whoami)
sudo chmod 666 /dev/spidev0.0       #센서 관련 설정
```
### 퍼블리셔 실행
```
ros2 run sensor_value_test sensor_publisher
```

### 근접센서
```
ros2 topic echo /proximity 
```
근접센서 True  : 30
근접센서 False : 40

### client 접속
```
ros2 topic echo /door_state
```
door_open :  1을 수신      10 퍼블리싱
door_close : 0을 수신      20 퍼블리싱
