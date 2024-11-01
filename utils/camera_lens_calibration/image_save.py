import cv2
from datetime import datetime

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 프레임을 읽을 수 없습니다.")
        break  # exit 대신 break로 변경하여 반복문 종료

    # cv2.imshow("camera", cv2.flip(frame, 1))
    cv2.imshow("camera", frame)

    key = cv2.waitKey(1) & 0xFF  # 여기서 & 0xFF로 처리
    if key == ord('q'):
        break
    elif key == ord('s'):
        cur_time = datetime.now()
        time_text = cur_time.strftime("%Y-%m-%d_%H-%M-%S")  # ':' 대신 '_'로 변경
        file_name = "imgs/" + time_text + ".jpg"
        cv2.imwrite(file_name, frame)
        print(f"{file_name} is saved.")

cap.release()
cv2.destroyAllWindows()
