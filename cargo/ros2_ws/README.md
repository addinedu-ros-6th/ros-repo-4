## 설치해야 할 모듈 
```
sudo apt update
sudo apt upgrade
sudo apt install python3-rpi.gpio
sudo pip3 install spidev

```

### 미니 봇에서 실행할 때 설정
```
sudo groupadd gpio
sudo groupadd spi
sudo usermod -aG gpio $USER
sudo chown root:$(whoami) /dev/gpiomem
sudo chmod 660 /dev/gpiomem



sudo chown root:gpio /dev/gpiomem 
sudo usermod -aG spi $(whoami)
sudo chmod 666 /dev/spidev0.0      
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
