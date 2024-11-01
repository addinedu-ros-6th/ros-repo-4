import cv2
import pickle
import numpy as np

# 캘리브레이션 데이터 로드
with open("data_cam160/calibration_data160.pkl", 'rb') as f:
    calibration_data = pickle.load(f)
    camera_matrix = calibration_data["camera_matrix"]
    dist_coeffs = calibration_data["dist_coeffs"]

# ArUco 사전 설정 (여기서는 기본적으로 DICT_4X4_50 사용)
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

# 웹캠 열기 (카메라 인덱스가 2인 경우)
cap = cv2.VideoCapture(3)

# 원하는 해상도로 설정
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

while True:
    # 프레임 캡처
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다. 종료합니다...")
        break

    # 프레임 왜곡 제거
    undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

    # ArUco 마커 탐지
    gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    # 마커가 감지되었는지 확인
    if ids is not None:
        # 각 마커에 대해 반복
        for i in range(len(ids)):
            # 마커의 코너 가져오기
            corner = corners[i]
            
            # 마커의 중심 좌표 계산
            cX = int((corner[0][0][0] + corner[0][2][0]) / 2.0)
            cY = int((corner[0][0][1] + corner[0][2][1]) / 2.0)
            
            # 마커의 방향 및 거리 계산
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, 0.05, camera_matrix, dist_coeffs)  # 마커 크기 0.05m
            (rvec - tvec).any()  # Numpy 오류 방지를 위한 코드
            distance = np.linalg.norm(tvec)  # 거리 계산

            # 마커의 위치와 방향을 그리기
            cv2.aruco.drawAxis(undistorted_frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
            cv2.aruco.drawDetectedMarkers(undistorted_frame, corners)
            
            # 마커 ID와 거리 텍스트 표시
            cv2.putText(undistorted_frame, f"ID: {ids[i][0]}, Distance: {distance:.2f}m", 
                        (cX, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 왜곡 제거된 프레임을 640x480으로 크기 조정
    resized_frame = cv2.resize(undistorted_frame, (1280, 720))
    
    # 크기 조정된 프레임을 표시
    cv2.imshow("Undistorted with ArUco", resized_frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 웹캠 및 윈도우 종료
cap.release()
cv2.destroyAllWindows()
