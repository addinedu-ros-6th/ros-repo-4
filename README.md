# ros-repo-4
파이널 프로젝트 4조 저장소. 공항 짐꾼 로봇 

![moti](https://github.com/user-attachments/assets/f7eb36ca-8dd6-4eb0-ac2c-85830dff3fa0)

## Contents

00. 프로젝트 성과물
01. About GOOGEESE  (팀원소개)
02. 프로젝트 기획
03. 프로젝트 설계
04. 프로젝트 기능
05. 적용기술
<br>

## 0. 프로젝트 성과물

### 0.1 시연영상 (유튜브 링크)
[![스크린샷 2024-11-28 110135](https://github.com/user-attachments/assets/2300ec6b-9408-4ea6-a2ee-b705e801d13c)](https://youtu.be/AxfbNDppkFc)

### 0.2 프로젝트 발표 자료 링크
(https://docs.google.com/presentation/d/1-NSuIJxcLXHOLSJyQHhXC_K7vsJ-sBmlO_trMq6Lffg/edit?usp=drive_link)

## 1. About GOOGEESE  (팀원소개)

### 김보성 (팀장)
* Jira, Confluence 개발 일정 및 문서 관리
* Following 모드 알고리즘 설계
* Following 모드 ROS 및 통신 패키지 구현
* 손동작(딥러닝)을 통한  일시정지 모드 구현

### 김주영
* 하드웨어 개발 환경 구축(Demo-map, 3D parts)
* SLAM 맵핑 및 GIMP 맵 보정
* 로봇 주행 테스트를 통한 NAV2 파라미터 최적화 및 Vector Pursuit 적용
* 관제소(Mother Goose)Admin GUI 설계 및 구현
* Admin GUI와 로봇들 간의 ROS통신 구현 

### 윤용곤
* AI Server & Client Web Server 구축
* ROS PKG - 서버 protocol 설계및 통신 구현
* State 전송 ROS PKG 설계 및 구현
* Client GUI 설계 및 구현 

### 이세형
* Behavior Tree 구조 설계 및 구현
* Navigation & Waypoints를 활용한 주행 기능 구현
* Aruco Marker를 활용한 주차 기능 구현
* 관제소(Mother Goose) GUI 화면 설계

### 임성렬
* 하드웨어 회로 Desing and Configuration
* GUI와 DB구축 및 연동 (저장 및 출력)
* GUI 연동 Topic으로 카고(서보모터) 제어 및 안정화
* 센서 값 Publish ( 카고 상태 및 수납 유무 )

## 2. 프로젝트 기획
### 2.1 일정
![sch](https://github.com/user-attachments/assets/548482d0-ff2a-4a36-b869-0f734d04a8bb)

### 2.1 협업 Confluence
![conf](https://github.com/user-attachments/assets/4f0d9953-99dc-499f-a6b5-7814cc773b29)

### 3.1 요구 기능 
#### 3.1.1 Driving Mode
<img src="https://github.com/user-attachments/assets/197d79a9-a81c-46b3-8c3e-44ef1c303188" alt="샘플 이미지" height="150">

* Auto Delivery : 로봇이 항공권 정보를 확인하고 탑승 게이트까지 알아서 짐을 배달
* Follow : 사용자를 따라오면서 짐을 실어주는 기능
* Pause at Stop Gesture : Follow 중 사용자가 잠시 따라오는 기능을 멈추고 싶을 때, 로봇에 멈춤 신호를 손동작으로 보내면 로봇이 해당 동작을 인식하고 멈추는 기능
* Return : 임무를 완수하고 출발지로 로봇이 알아서 이동하는 기능

#### 3.1.2 Cargo Security
<img src="https://github.com/user-attachments/assets/007b1fce-2db6-4920-8f30-3ccefaf6ebcd" alt="샘플 이미지" height="150">

* Cargo Lock / Unlock : 운송 중 화물은 밀폐 공간에 안전하게 보관되어야 함. 
* Face Recognition : 화물을 찾을 때는 안면 인증 방식으로 보안성이 뛰어나고 편리해야 함. 

#### 3.1.3 Admin System
<img src="https://github.com/user-attachments/assets/3b0a11da-64ab-4abf-8340-93f2104df68c" alt="샘플 이미지" height="150">

* Robot State Monitoring : 운행 중인 모든 로봇의 위치, 상태 등의 정보가 모니터링 되어야 함. 
* Admin Emergency Stop : 긴급상황 발생시, 관제 센터에서 각 로봇의 운행을 정지 시킬 수 있어야 함. 

### 3.2 Operating Scenario
![scenario1](https://github.com/user-attachments/assets/2a2071af-91df-4fb5-b0c6-d1760eb2168e)

## 3. 프로젝트 설계
### 3.1 System Architecture
![system_architecture](https://github.com/user-attachments/assets/f5710d4a-1c00-4a2d-a854-46ec0dbbbc66)

### 3.2 State Machine
![state_machine](https://github.com/user-attachments/assets/c823c2f7-a666-42df-b3ac-c2424b254c65)

### 3.3 ERD
![ERD](https://github.com/user-attachments/assets/7b8b92bb-639b-4f48-a3f8-cadfbd96f50a)

### 3.4 H/W 개발 환경 구축
* Demo Map
![demo_map](https://github.com/user-attachments/assets/1d160134-4ea9-4ae5-8b45-c49dac7460d2)
* Baby Goose Cargo Parts
![Cargo_parts](https://github.com/user-attachments/assets/9f670868-8a01-4761-906f-611641484729)

### 3.5 기술 스택
![기술스택](https://github.com/user-attachments/assets/a7fc6008-3829-4968-a419-c5c6d665ebaf)

## 4. 프로젝트 기능
### 4.1 사용자 등록 /  짐싣기 / 주행모드 결정
![기능1등록](https://github.com/user-attachments/assets/d0e2ea97-74b8-4925-860f-88def0d14ce6)

* Client GUI 접속
    - 사용자는 공항에서 출국심사를 마치고 출국장에 나오면 주변에 정차된 로봇의 QR 을 스캔한다.
    - QR 에는 해당 로봇 ID 와 client web service 주소가 포함되어 있어, 스마트폰으로 해당 서비스에 바로 접속하며, 서버에는 해당 client 에 해당 로봇 ID 로 session ID 를 발급한다. 
    - Web Server 는 해당 Robot ID 와 Client 요청 상태를 State Controller 에 전달한다. 
    - State Controller 는 해당 내용을 `/Request` topic 으로 publish 함으로써 이를 구독하고 있는 모든 ROS node (ex. `Admin GUI`) 로 현재 사용하고 있는 로봇의 사용 정보가 공유 된다.
* 사용자 얼굴 등록
    - 핸드폰 카메라를 사용하여 본인 사진을 촬영한다. 이는 나중에 도착지에서 수화물을 찾을 때 인증 얼굴과 동일 인물임을 확인하기 위해 사용된다. 
* 사용자 항공권(QR) 등록
    - 전자 항공권 QR 을 스마트폰 카메라에 보여주면, 자동으로 인식하여 해당 정보를 확인한다.
* 입력 정보 확인
    - 얼굴 사진과 항공권 정보가 맞는지 다시 한번 확인. 
    - 틀리다면 No 를 눌러서 초기화면으로 되돌아 간다.
    - 맞다면 Yse 를 눌러 짐싣기 화면으로 이동한다. 동시에 해당 정보는 DB 에 저장된다. 
* 짐싣기
    - 해당 페이지로 이동하면 바로 Cargo Controller Node 에 의해 cargo 문이 열린다. 
    - Corgo Controller 는 현재 근접 센서에 의한 화물 적재 여부, servor motor 에 의한 문 열림/닫힘 상태를 `/Response` topic 으로 발행하여 State Controller 에서 현재 Cargo 상태를 확인, 해당 정보를 Client GUI Web Server 에 전송하여 GUI 상 상태를 표시해 준다. 
    - 짐을 싣고나서, 닫힘버튼을 누르면 Cargo 는 닫히게 된다. 
    - 문이 완전히 닫히면, 닫힘 상태가 State Controller 에 확인되면 Client GUI 다음 페이지로 이동한다.
* 사용자 모드 선택
    - Auto Delivery : 로봇이 알아서 비행기 Gate 까지 이동
    - Follow : 사용자 뒤를 쫓아가면 짐을 싣고 이동
  
### 4.2 주행
### 4.2.1 Auto Delivery
* 다음의 구현 기술을 사용하여 개선된 자율 주행 성능으로 목적지까지 이동 (기술에 대한 상세한 설명은 5번 항목에 기재됨.)
    - Nav2 Path-Tracking Algorithm  Vector Pursuit Plug-in 적용및 파라미터 최적화 
    - Waypoint 적용으로 좁은 구역 통과 성능 개선
    - 최종 목적지 Aruco Marker 인식 및 위치 조정를 통한 도착 위치 정밀도 개선
### 4.2.2 Follow
*  딥러닝 Mediapipe 을 사용하여 실시간으로 검출된 landmark 정보를 사용하여, 신속한 객체 인식과 정확한 이동 알고리즘 구현 
*  사람이 로봇의 움직임을 일시 정지하기 위해 손바닥을 보여 정지 요청을 하면 이를 인식하여 정지
*  (기술에 대한 상세한 설명은 5번 항목에 기재됨.)
<img src="https://github.com/user-attachments/assets/40b9d320-d5a9-4a80-8545-a9b69fb5ebf9" alt="" width="500">

### 4.3 목적지 도착 / 짐 꺼내기 / 사용 종료
![기능3사용완료](https://github.com/user-attachments/assets/db64da2c-f7b5-4b5c-8525-d3bffe53e7f0)

* 사용자 얼굴 재확인
    - 도착지에서 최조 등록 사용자 얼굴과 일치하는지 확인한다. 
    - 사용자가 일치하는 경우에만 짐을 찾을 수 있다. 
    - 관련 딥러닝 기술로, YOLO face 나 Deepface 를 사용하여 내부적으로 처리 가능함을 확인하였으나, 기술조사 단계에서 현장의 배경, 조도, 각도, 닮은꼴 이미지 등으로 확인하였을 때, 가장 좋은 결과를 보인 AWS Rekognition API 를 사용하는 것으로 결정함.
* 짐 꺼내기
    - 해당 페이지에 이동하면서 바로 Cargo Controller node 에 의해 문은 열리게 된다. 
    - 해당 작업 내부 프로세스는 앞에서 설명한 `짐싣기` 와 동일하게 동작
* 사용 종료
    - 사용자가 사용 종료 버튼을 누르면 로봇은 `return` 모드로 전환되어 출발 위치로 돌아감.

### 4.4 관제센터 (Admin Gui)
![mothergui](https://github.com/user-attachments/assets/528a61a3-a751-4683-aad5-aed7cb3b2718)

#### 4.4.1 로봇 위치 및 상태 표시
* 화면 상단에 공항 전체 Map 과 현재 로봇들의 위치를 표시함. 
* 화면 하단에는 각 로봇의 다음 상태를 확인가능
    - 공항에 배치 동작 중인 로봇
    - 로봇 Motor 상태 
    - 로봇 Lidar 상태 
    - 사용자 로봇 이용 정보 (state) 
#### 4.4.2 로봇 영상 실시간 확인 및 긴급제동
* 화면 상단에 각 로봇 전송하는 영상을 확인 가능
* 각 로봇의 운행을 멈추게 하는 하단 버튼

## 5. 적용기술
### 5.1 Deeplearning
#### 5.1.1. Person Tracking
![스크린샷 2024-12-03 160124](https://github.com/user-attachments/assets/db94e02c-82c7-4bc5-b5c1-e30e0efb15af)

1. 기술 조사
* Tracker 모델 사용 
  - YOLO Tracker, Deepsort 등의 딥러닝 모델을 사용하여 객체 추적을 진행하는 경우, Frame 내 모든 인물에 대한 ID 가 부여됨. 
  - 특정 ID 객체만 추적하기 위해서는 해당 객체의 특징(ex. 얼굴, 의상 등)을 다시 검출하고 해당 정보가 일치하는 객체을 확인 해야 하므로, 얼굴 인식, 의상 검출 등의 추가 모델을 함께 동작해야 함.
  - 이렇게 다중 모델 적용시, 추적 인물에 대한 상세 정보를 확인하여 선별적으로 추적을 진행할 수 있음.
  - 그러나 현재 H/W 성능상 이렇게 복합적으로 구현했을 때, 처리시간이 10 FPS 이하로 떨어져서 매 loop 당 로봇이 추적해야 하는 변위가 커지며, Deepsort 모델 내 칼만 필터에 의해 예측된 다음 위치 값에 대한 정확도가 매우 떨어져 추적이 어려워짐을 확인.        
  - 아울러 관련 객체 인식의 경우, 모든 신체을 포함한 bounding box 정보를 확보하게 되는데 팔을 벌리거나, 신체를 숙이는 등의 동작에서 bounding box 의 중심좌표가 실제 신체의 무게중심에서 크게 벗어나는 현상이 발생하여, 이동 중심 좌표 검출을 위한 추가 모델 검토가 필요할 수 있음을 확인함.

* 신체 탐지 및 추적 data 확보를 위한 단일 & 경량화 모델 사용 (Mediapipe)
    - 추적 로봇 실제 사용 환경을 고려 시, 1미터 이내 간격을 유지하여 사람을 추적해야 하므로, 실제 사용환경에서 사용자가 로봇의 영상에서 일반적으로 중심의 위치에 가장 큰 객체로 인식됨을 확인함 (2024 로보월드)
    - 해당 사용 조건에서 Mediapipe pose 모델의 경우 중심위치의 가장 큰 단일 객체만 자동으로 인식하므로 다중 모델 적용 없이 (현 프로젝트 기준) 적용 가능함을 확인함.
    - 좌우 어깨 와 골판 landmark 좌표를 통하여, 인물 상체 무게 중심 및 상대적 인물 크기를 계산
    - 상대적 인물 사이즈 및 위치 정보를 사용하여, 로봇의 상대 위치를 조정하도록 구현함. 

#### 5.1.2 Stop Gesture
![스크린샷 2024-12-03 162633](https://github.com/user-attachments/assets/0b6aec81-a7e6-4919-8f48-70754e8e1f7e)

* Mediapipe Hand  학습 데이터를 사용한 Decision Tree 생성 (classification : ‘pause’, ‘opposite_pause’, ‘fist’)
* ‘pause’  class 인식된 동작에 대해 현재 조건에 맞게 추가 알고리즘 적용 
  - 좀더 명시적으로 카메라에 손가락을 펴고, 손바닥을 보이는 자세를 취했을 때만 동작하도록 pause 상태를 다음과 같은 상태로 제한시킴.
    - 손목 landmark (0) 이 로봇이 따라가는 무게 중심 좌표보다 높으며,
    - 다섯 손가락이 모두 위쪽을 향해 펴져 있는 경우

### 5.2 Path-Tracking Algorithm
#### Vector Pursuit Controller
* Limitation of NAV2
![vector_persuit1](https://github.com/user-attachments/assets/c80ba92c-c24b-418d-8d8a-116698b22063)

* Task: Tuning Parameters
![vector_persuit2](https://github.com/user-attachments/assets/6dc6ee9e-9ec3-4700-ab10-885227eb1353)

* Limitation of DWB Local Planner
![DWB_8x](https://github.com/user-attachments/assets/65d1c8ba-ab03-4ebc-883f-d52014d1acf7)

1. 정확한 경로 추적

* Task: Alternative to DWB, Vector Pursuit
![DWB_vs_vector](https://github.com/user-attachments/assets/6aea2950-01aa-41e4-a587-93a9d2dd1335)

(영상 링크)
[![스크린샷 2024-11-29 100729](https://github.com/user-attachments/assets/4b751a7c-f9f2-4737-a179-dd8ac8990596)](https://www.youtube.com/watch?v=FzJ_Zio0tOg)

2. 적극적인 충돌 감지
![스크린샷 2024-11-29 100624](https://github.com/user-attachments/assets/135e9c59-9c9a-4e87-9db0-5b99a2507773)

![obstacle_avoiding](https://github.com/user-attachments/assets/a96884fb-3070-4b3e-a2dd-0c147f05c1a9)
![DWB_vs_vector_fps10_scale640](https://github.com/user-attachments/assets/243d549d-21bc-4471-b40c-65bf1145de54)

* 추후 개선 고려 사항
![try](https://github.com/user-attachments/assets/7217e787-22d1-4d1d-a35c-bfa42754c15a)

####  Navigate To Waypoints In Behavior Tree

* Navigation Tree Structure
![Navigation Tree Structure](https://github.com/user-attachments/assets/f095783c-b4a3-4fa8-bd6c-57c95b02be66)

#### Aruco Marker Following In Behavior Tree
* Aruco Following Tree Structure
![스크린샷 2024-11-29 103341](https://github.com/user-attachments/assets/c48e892d-c570-475e-9fb0-d3763ba9777b)
* 주행 영상
![aruco_detail](https://github.com/user-attachments/assets/2f71b48a-09c3-47a3-8250-19c0d006e9ac)

#### Navigation & Aruco Marker Following Demo Video
![Path-Tracking](https://github.com/user-attachments/assets/35dfbdd8-e667-4bf3-810a-e89ee92bac70)