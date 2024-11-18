import cv2
import numpy as np
import pickle

# 캘리브레이션 데이터 로드
try:
    with open("/home/sehyung/minibot_ws/src/pinklab_minibot_robot/aruco_follower/aruco_follower/calibration_minibot5.pkl", 'rb') as f:
        calibration_data = pickle.load(f)
        camera_matrix = calibration_data["camera_matrix"]
        dist_coeffs = calibration_data["dist_coeffs"]
except Exception as e:
    print("Calibration file error:", e)
    camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]])  # 기본값 설정
    dist_coeffs = np.zeros((5, 1))  # 왜곡 계수를 0으로 설정

# ArUco 마커 설정
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()
marker_length = 0.05  # 마커의 실제 크기 (미터 단위)

def rotation_matrix_to_euler_angles(rotation_matrix):
    # 회전 행렬을 오일러 각도로 변환
    sy = np.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        y = np.arctan2(-rotation_matrix[2, 0], sy)
        z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        x = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        y = np.arctan2(-rotation_matrix[2, 0], sy)
        z = 0
    return np.array([x, y, z])

# 웹캠 열기
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 프레임을 흑백으로 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # ArUco 마커 탐지
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    if ids is not None:
        for i in range(len(ids)):
            # 마커의 위치와 회전 추정
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_length, camera_matrix, dist_coeffs)

            # 회전 벡터를 회전 행렬로 변환
            rotation_matrix, _ = cv2.Rodrigues(rvec[0][0])

            # roll, pitch, yaw 계산
            angles = rotation_matrix_to_euler_angles(rotation_matrix)
            roll, pitch, yaw = np.degrees(angles)

            # 마커와 축 그리기
            cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec[0][0], tvec[0][0], 0.05)
            cv2.aruco.drawDetectedMarkers(frame, corners)

            # 각도 정보 표시
            cX = int((corners[i][0][0][0] + corners[i][0][2][0]) / 2.0)
            cY = int((corners[i][0][0][1] + corners[i][0][2][1]) / 2.0)
            cv2.putText(frame, f"Roll: {roll:.2f}", (cX, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"Pitch: {pitch:.2f}", (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"Yaw: {yaw:.2f}", (cX, cY + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # 결과 출력
    cv2.imshow("ArUco Marker Detection", frame)
    
    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
