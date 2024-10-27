import cv2
import pickle
import glob

distorted_images = glob.glob("distorted_imgs160/*.jpg")

with open("calibration_data160.pkl", 'rb') as f:
    calibration_data = pickle.load(f)
    camera_matrix = calibration_data["camera_matrix"]
    dist_coeffs = calibration_data["dist_coeffs"]


# 왜곡된 이미지 불러오기
for fname in distorted_images:
    file_name = fname.split("/")[1]
    img = cv2.imread(fname)
    
    undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs)

    file = "undistorted_imgs160/" + file_name
    cv2.imwrite(file, undistorted_img)