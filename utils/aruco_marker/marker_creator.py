import cv2
import cv2.aruco as aruco


# 생성할 Aruco 마커의 딕셔너리 타입과 ID를 설정
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)  # DICT_4X4_50은 4x4 그리드와 50개의 ID 세트를 의미
marker_size = 500  # 마커의 크기 (픽셀)

# Aruco 마커 생성
for marker_id in range(1, 20):
    marker_image = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

    # 마커 이미지를 파일로 저장
    cv2.imwrite(f"imgs/aruco_marker_{marker_id}.png", marker_image)

# 생성된 마커를 화면에 표시
cv2.imshow("Aruco Marker", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()