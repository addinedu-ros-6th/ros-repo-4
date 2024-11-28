# ros-repo-4
파이널 프로젝트 4조 저장소. 공항 짐꾼 로봇 

![moti](https://github.com/user-attachments/assets/f7eb36ca-8dd6-4eb0-ac2c-85830dff3fa0)

## Contents

00. 시연영상
01. About GOOGEESE  (팀원소개)
02. 프로젝트 기획
03. 프로젝트 설계
04. 프로젝트 기능
05. 적용기술
<br>

## 0. 시연영상 (유튜브 링크)
[![스크린샷 2024-11-28 110135](https://github.com/user-attachments/assets/2300ec6b-9408-4ea6-a2ee-b705e801d13c)](https://youtu.be/AxfbNDppkFc)

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
![스크린샷 2024-11-28 173759](https://github.com/user-attachments/assets/197d79a9-a81c-46b3-8c3e-44ef1c303188)
* Auto Delivery : 로봇이 항공권 정보를 확인하고 탑승 게이트까지 알아서 짐을 배달
* Follow : 사용자를 따라오면서 짐을 실어주는 기능
* Pause at Stop Gesture : Follow 중 사용자가 잠시 따라오는 기능을 멈추고 싶을 때, 로봇에 멈춤 신호를 손동작으로 보내면 로봇이 해당 동작을 인식하고 멈추는 기능
* Return : 임무를 완수하고 출발지로 로봇이 알아서 이동하는 기능

#### 3.1.2 Cargo Security
<img src="https://github.com/user-attachments/assets/007b1fce-2db6-4920-8f30-3ccefaf6ebcd" alt="샘플 이미지" width="220">

* Cargo Lock / Unlock : 운송 중 화물은 밀폐 공간에 안전하게 보관되어야 함. 
* Face Recognition : 화물을 찾을 때는 안면 인증 방식으로 보안성이 뛰어나고 편리해야 함. 

#### 3.1.3 Admin System
<img src="https://github.com/user-attachments/assets/3b0a11da-64ab-4abf-8340-93f2104df68c" alt="샘플 이미지" width="220">

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
## 5. 적용기술
### 5.1 Deeplearning
#### 5.1.1. Person Tracking

#### 5.1.2 Stop Gesture

### 5.2 Path-Tracking Algorithm
#### Vector Pursuit Controller
* Limitation of NAV2
![vector_persuit1](https://github.com/user-attachments/assets/c80ba92c-c24b-418d-8d8a-116698b22063)
<관련 설명 명시 요망>

* Task: Tuning Parameters
![vector_persuit2](https://github.com/user-attachments/assets/6dc6ee9e-9ec3-4700-ab10-885227eb1353)
<관련 설명 명시 요망>
