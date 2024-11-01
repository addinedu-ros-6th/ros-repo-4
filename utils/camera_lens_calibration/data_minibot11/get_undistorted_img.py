import cv2
import pickle
import glob

before_cal_dir =  "before_calibration/*.jpg"
distorted_images = glob.glob(before_cal_dir)

pkl_dir = "calibration_minibot11.pkl"
with open(pkl_dir, 'rb') as f:
    calibration_data = pickle.load(f)
    camera_matrix = calibration_data["camera_matrix"]
    dist_coeffs = calibration_data["dist_coeffs"]


# 왜곡된 이미지 불러오기
for fname in distorted_images:
    file_name = fname.split("/")[-1]
    img = cv2.imread(fname)
    
    undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs)

    file = "after_calibration/" + file_name
    cv2.imwrite(file, undistorted_img)