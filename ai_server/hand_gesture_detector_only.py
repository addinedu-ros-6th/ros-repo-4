import cv2
import mediapipe as mp
import pickle
import numpy as np
import pandas as pd
from sklearn.preprocessing import LabelEncoder

# 저장된 모델 로드
with open('best_model.pkl', 'rb') as f:
    model = pickle.load(f)

# 레이블 인코딩
labels = ['pause', 'opposite_pause', 'fist']
le = LabelEncoder()
le.fit(labels)

# Mediapipe 손 인식 모델 초기화
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# 웹캠 초기화
cap = cv2.VideoCapture(0)

# 랜드마크 인덱스
landmark_indices = [0, 4, 8, 12, 16, 20, 1, 5, 9, 13, 17]

with mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5) as hands:

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        # BGR 이미지를 RGB로 변환
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # 성능을 향상시키기 위해 이미지 쓰기 금지 설정
        image.flags.writeable = False
        # 이미지 처리
        results = hands.process(image)
        # 이미지 쓰기 허용 설정
        image.flags.writeable = True
        # RGB 이미지를 다시 BGR로 변환
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # 랜드마크 그리기 및 예측
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # 랜드마크 추출 (중요 랜드마크만)
                landmarks = []
                for i in landmark_indices:
                    lm = hand_landmarks.landmark[i]
                    landmarks.append(lm.x)
                    landmarks.append(lm.y)
                landmarks = np.array(landmarks).flatten()

                # 예측
                landmarks_df = pd.DataFrame([landmarks])
                gesture = model.predict(landmarks_df)[0]
                gesture_name = le.inverse_transform([gesture])[0]

                # 예측 결과를 이미지에 표시
                cv2.putText(image, f'Order: {gesture_name}', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

        # 이미지 출력
        cv2.imshow('Hand Landmarks', image)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

# 웹캠 해제 및 모든 창 닫기
cap.release()
cv2.destroyAllWindows()
