import cv2

# QRCodeDetector 초기화
detector = cv2.QRCodeDetector()

# 웹캠 열기 (기본적으로 0번 카메라)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Press 'q' to exit.")

while True:
    # 웹캠에서 프레임 읽기
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # QR 코드 디코딩
    retval, decoded_info, points, _ = detector.detectAndDecodeMulti(frame)
    if retval:
        for data in decoded_info:
            if data:
                print(f"Decoded QR Code: {data}")

    # QR 코드 영역 표시
    # QR 코드 영역 표시
    if points is not None:
        for point in points:
            if point is not None:
                # QR 코드의 꼭짓점들을 정수로 변환
                pts = point.astype(int)
                for i in range(len(pts)):
                    pt1 = tuple(pts[i])  # numpy.int64를 tuple로 변환
                    pt2 = tuple(pts[(i + 1) % len(pts)])
                    cv2.line(frame, pt1, pt2, (0, 255, 0), 2)


    # 프레임 표시
    cv2.imshow("QR Code Scanner", frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 리소스 해제
cap.release()
cv2.destroyAllWindows()
