import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import pickle

BOARD_NUM = "3"
SAVE_DIR = "calibrated_chessboard160/"

# size unit : mm
chessboard = {
    "1" : {
        "rows" : 5,
        "columns" : 7,
        "size" : {
            "chess" : 50.00
        }
    },
    "2" : {
        "rows" : 6,
        "columns" : 8,
        "size" : {
            "chess" : 42.40
        }
    },
    "3" : {
        "rows" : 7,
        "columns" : 9,
        "size" : {
            "chess" : 39.25
        }
    }   
}

# termination criteria
# cv2.TERM_CRITERIA_EPS : 위치 변화가 설정된 오차보다 작으면 종료
# cv2.TERM_CRITERIA_MAX_ITER: 설정된 반복 횟수만큼 반복한 경우 종료
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
pattern_size = (chessboard[BOARD_NUM]["rows"] - 1, chessboard[BOARD_NUM]["columns"] - 1) # (체스보드 가로 패턴 개수 - 1, 체스보드 세로 패턴 개수 - 1)
square_size = chessboard[BOARD_NUM]["size"]["chess"]

objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob("origin_chessboard160/*.jpg")

for fname in images:
    file_name = fname.split("/")[1]
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)

        # cv2.cornerSubPix(1, 2, 3, 4, 5)
        # 체스보드의 코너를 더욱 정확하게 위치시키기 위해 사용하는 함수
        # 이 함수는 cv2.findChessboardCorners()로 대략적으로 찾은 코너들을 세밀하게 조정하여 
        # 서브픽셀(sub-pixel) 수준에서 정확한 좌표를 반환. 
        # 여기서 서브픽셀은 픽셀 단위보다 더 작은 해상도에서 좌표를 계산하는 방법을 의미
        # 
        # 파라미터
        # 1: 입력 이미지로, 그레이스케일 이미지가 필요
        # 2: 대략적으로 찾은 코너들의 초기 좌표 cv2.findChessboardCorners() 함수가 
        #    반환한 좌표들이며, 이를 더 세밀하게 조정하는 과정에서 사용됨. 
        #    체스보드의 각 코너에 대한 좌표 목록
        # 3: 서치 윈도우 크기
        #    서브픽셀 정확도로 코너를 찾기 위해 주변에서 탐색할 검색 창의 크기 (width, height)
        #    (11, 11)은 해당 코너 주변 11x11 픽셀의 영역 내에서 서브픽셀 정확도로 코너를 세밀하게 조정하겠다는 의미
        # 4: 무효 영역의 크기
        #    (-1, -1)이면 서치 윈도우 전체가 사용된다는 의미입니다. 즉, 코너 주변의 모든 영역을 탐색에 활용
        # 5: 반복 종료 기준
        #    코너 위치를 찾는 반복 알고리즘의 종료 조건
        #    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER는 주어진 반복 횟수(30번) 내에, 
        #    설정된 허용 오차(0.001)만큼 코너 위치가 변화하지 않으면 계산을 종료하라는 의미
        #
        # 반환값 : 서브픽셀 정확도로 조정된 코너 위치 좌표를 반환
        #         corners2는 코너들의 서브픽셀 좌표로, corners에 비해 더 정확한 값을 갖음.
        #         코너 위치가 정교하게 조정되었으므로, 이 값을 사용하면 보정 작업에서 더 나은 결과를 얻을 수 있다.
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, pattern_size, corners2, ret)
        save_file = SAVE_DIR + file_name
        cv2.imwrite(save_file, img)
        print("Image saved successfully.")
    else:
        print("Not found the chess board corners")

# 카메라 보정 수행
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# 결과 출력
print("Camera metrix:")
print(camera_matrix)

print("\nDistortion coefficients:")
print(dist_coeffs)

# 보정 결과 저장
with open("calibration_data160.pkl", "wb") as f:
    pickle.dump({
        "camera_matrix": camera_matrix,
        "dist_coeffs": dist_coeffs,
        "rvecs": rvecs,
        "tvecs": tvecs
    }, f)




        