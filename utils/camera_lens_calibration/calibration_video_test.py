import cv2
import pickle

# Load calibration data
with open("calibration_data160.pkl", 'rb') as f:
    calibration_data = pickle.load(f)
    camera_matrix = calibration_data["camera_matrix"]
    dist_coeffs = calibration_data["dist_coeffs"]

# Open the default webcam (change the index if you have multiple cameras)
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("Cannot open webcam")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # If frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Undistort the frame
    undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

    # Optionally, you can resize the frames for better display
    # frame = cv2.resize(frame, (640, 480))
    # undistorted_frame = cv2.resize(undistorted_frame, (640, 480))

    # Stack the original and undistorted frames side by side
    combined_frame = cv2.hconcat([frame, undistorted_frame])

    # Display the resulting frame
    cv2.imshow('Original (Left) vs Undistorted (Right)', combined_frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture and close windows
cap.release()
cv2.destroyAllWindows()
