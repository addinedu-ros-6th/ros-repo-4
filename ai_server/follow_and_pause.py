import cv2
import pickle
import mediapipe as mp
import numpy as np
import pandas as pd
from sklearn.preprocessing import LabelEncoder

# landmark visibility 임계값 설정
VISIBILITY_THRESHOLD = 0.7
INITIAL_BIAS_LIMIT = 60
INITIAL_BODY_SIZE = 4000
MIN_BODY_SIZE = 1500
MAX_FAILED_COUNTER = 20

HAND_LABELS = ['pause', 'opposite_pause', 'fist']

#       8 12 16  20
#       |  |  |  |
#   4   |  |  |  |
#    \  5  9  13 17
#     \
#       1 
#            0 
HAND_LANDMARKS_INDICES = [0, 4, 8, 12, 16, 20, 1, 5, 9, 13, 17]


class Follower():
    def __init__(self):
        # Mediapipe Pose  모듈 초기화
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        # ------ for body detector -----
        self.sub_mode = 0  # 0: stop & searching 1: too far 2: move to left 3: move to right 4: following
        self.failed_counter = 0

        self.img = None
        self.body_center = [0, 0]
        self.body_size = 0

        self.cur_center = [0, 0]
        self.cur_body_size = 0

        self.gesture_name = "Others"

        self.pose = mp.solutions.pose.Pose(
            min_detection_confidence=0.7, 
            min_tracking_confidence=0.7
            )
        # --------------------------------
        
        # ------ for hand detector -----
        self.mp_hands = mp.solutions.hands

        with open('hand_model.pkl', 'rb') as f:
            self.hand_model = pickle.load(f)

        # 레이블 인코딩
        self.le = LabelEncoder()
        self.le.fit(HAND_LABELS)

        self.hands = self.mp_hands.Hands(
            max_num_hands=1, 
            min_detection_confidence=0.7, 
            min_tracking_confidence=0.5
            )
    
        self.hand = "Others"
        # --------------------------------
    
    def run(self, image):
        """
        화면 속 인물의 크기 위치를 확인하여 인식/추적 모드 전환 및 관련 data 를 반환
        추적 모드인 경우, 사람의 손동작을 인식하여 멈춤 기능을 포함.
        """
        h, w, c = image.shape
        center_x = w // 2
        center_y = h // 2

        # 사각형 좌표를 시계 방향으로 지정
        p1 = [0, 0]     # idx : 11 - left shoulder 
        p2 = [0, 0]     # idx : 12 - right shoulder
        p3 = [0, 0]     # idx : 24 - right hip
        p4 = [0, 0]     # idx : 23 - left hip
        
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # 성능을 향상 시키기 위해 이미지를 쓰기 불가능으로 설정
        image.flags.writeable = False
        results = self.pose.process(image)            
        image.flags.writeable = True

        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        image = cv2.circle(image, (center_x, center_y), 5, (255, 0, 0), -1) # 프레임 중심점 표시
        
        #  포즈 랜드마크를 이미지 위에 그리기
        if results.pose_landmarks:
            for idx, landmark in enumerate(results.pose_landmarks.landmark):
                # 임계값 이상의 landmark 만 사용
                if landmark.visibility > VISIBILITY_THRESHOLD:
                    x = int(landmark.x * w)
                    y = int(landmark.y * h)
                    cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
                    # cv2.putText(image, f"{idx}", (x, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
                    if idx == 11:
                        p1 = [x, y]
                    elif idx == 12:
                        p2 = [x, y]
                    elif idx == 24:
                        p3 = [x, y]
                    elif idx == 23:
                        p4 = [x, y]
            # mp_drawing.draw_landmarks(
            #     image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS, 
            #     mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
            #     mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
            # )

            # 좌우 어깨.골반 좌표의 무게중심 계산
            self.cur_center[0] = (p1[0] + p2[0] + p3[0] + p4[0]) // 4
            self.cur_center[1] = (p1[1] + p2[1] + p3[1] + p4[1]) // 4

            # 두 삼각형 벡터 곱의 값을 산출 이를 더하여 사각형 면적 계산
            self.cur_body_size = 0.5 * (abs(p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1])) 
                                        + abs(p1[0] * (p3[1] - p4[1]) + p3[0] * (p4[1] - p1[1]) + p4[0] * (p1[1] - p3[1])))

            if self.sub_mode < 4  or self.cur_body_size > MIN_BODY_SIZE: 
                self.failed_counter = 0
            else:
                self.failed_counter += 1      
        else:
            self.failed_counter += 1 

        # ------------ 최초 인식 단계 ------------------
        if self.sub_mode < 4:
            self.body_center = self.cur_center
            self.body_size = self.cur_body_size
            
            # 화면내 상체 크기가 일정 이상 확보된 경우
            if self.body_size > INITIAL_BODY_SIZE:
                if self.body_center[0] - (center_x - INITIAL_BIAS_LIMIT) < 0:
                    self.sub_mode = 2
                    self.img = cv2.putText(image, "Please move to left side.", (16, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
                elif self.body_center[0] - (center_x + INITIAL_BIAS_LIMIT) > 0:
                    self.sub_mode = 3
                    self.img = cv2.putText(image, "Please move to right side.", (16, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
                else: 
                    self.img = image
                    self.sub_mode = 4
                    cv2.putText(image, "I recognized you! Let's go!", (16, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
            # 화면 내 상체 크기가 일정 이하인 경우
            else:
                self.sub_mode = 1
                self.img = cv2.putText(image, "Please come a little closer.", (16, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
        
        # ---- 주행 단계 (인식 후 following or guiding) ----
        elif self.sub_mode == 4:
            # 정상적으로 현재 프레임에서 사람이 확인(검출)된 상태
            if self.failed_counter == 0:
                self.body_center = self.cur_center
                self.body_size = self.cur_body_size
                self.img = image

                # 현재 프레임 기준 손동작을 멈춤 요청이 있는지 확인
                hand_check_img, self.hand = self.hand_detector(image)

                image = cv2.putText(hand_check_img, "Following", (16, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
                
            # 현재 프레임에서 사람이 검출되지 않았지만 아직 failed_counter 가 제한을 넘지않아.
            # 이전에 저장된 프레임을 사용하여 추적하고 있는 상태
            elif self.failed_counter <= MAX_FAILED_COUNTER:
                self.img = cv2.putText(self.img, "Following(prev_frame)", (16, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
            else:                    
                self.follower_reset(image)
                
        # ----------------------------------------------
        
        diff_x = center_x - self.body_center[0]
        diff_y = center_y - self.body_center[1]
        
        # 상체 무게 중심 좌표 표시                
        text = f"mode: {self.sub_mode}, diff_x: {diff_x}, diff_y: {diff_y}, body_size: {self.body_size}, counter : {self.failed_counter}" 
        send_img = cv2.circle(image, (self.body_center[0], self.body_center[1]), 7, (0, 0, 255), -1)
        send_img = cv2.putText(image, text, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv2.LINE_AA)

        return send_img, self.sub_mode, diff_x, diff_y, self.body_size, self.hand


    def hand_detector(self, image):
        h, w, c = image.shape

        if image.dtype != np.uint8:
            image = image.astype(np.uint8)
        
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # 성능을 향상시키기 위해 이미지 쓰기 금지 설정
        image.flags.writeable = False
        # 이미지 처리
        results = self.hands.process(image)
        # 이미지 쓰기 허용 설정
        image.flags.writeable = True
        # RGB 이미지를 다시 BGR로 변환
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # 랜드마크 그리기 및 예측
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                
                # 랜드마크 추출 (중요 랜드마크만)
                landmarks = []
                for i in HAND_LANDMARKS_INDICES:
                    lm = hand_landmarks.landmark[i]
                    landmarks.append(lm.x)
                    landmarks.append(lm.y)                

                landmarks = np.array(landmarks).flatten()

                # 예측
                landmarks_df = pd.DataFrame([landmarks])
                gesture = self.hand_model.predict(landmarks_df)[0]
                self.gesture_name = self.le.inverse_transform([gesture])[0]

                if self.gesture_name == HAND_LABELS[0]:
                    # 모델 학습 결과가 pause 인 경우에 대해 
                    # 손목이 상체 무게 중심점 보다 높게 위치하고
                    # 모든 손가락이 (위로 향해)펴져 있을 때만 pause 로 다시 filtering  함.
                    # 그외 경우는 others 로 처리 
                    if  self.body_center[1] > hand_landmarks.landmark[0].y * h\
                        and hand_landmarks.landmark[1].y > hand_landmarks.landmark[4].y\
                        and hand_landmarks.landmark[5].y > hand_landmarks.landmark[8].y\
                        and hand_landmarks.landmark[9].y > hand_landmarks.landmark[12].y\
                        and hand_landmarks.landmark[13].y > hand_landmarks.landmark[16].y\
                        and hand_landmarks.landmark[17].y > hand_landmarks.landmark[20].y:
                        pass
                    else:
                        self.gesture_name = "Others"

                # 예측 결과를 이미지에 표시
                cv2.putText(image, f'Hand: {self.gesture_name}', (16, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1, cv2.LINE_AA)

        return image, self.gesture_name

        
    def follower_reset(self, image):
        self.sub_mode = 0  
        self.failed_counter = 0

        self.img = image
        self.body_center = [0, 0]
        self.body_size = 0

        self.cur_center = [0, 0]
        self.cur_body_size = 0
            


# 자체 영상 테스트 시 활성화하여 사용
def main():
    follower = Follower()

    cap = cv2.VideoCapture(0)
    
    while cap.isOpened():
        ret, image = cap.read()
        h, w, c = image.shape
        print(f"hieght: {h // 2}, width : {w // 2}")
        if not ret:
            break

        img, sub_mode, diff_x, diff_y, body_size = follower.run(image)
    
        # img = follower.hand_detector(image)
        # img = cv2.circle(img, (w // 2, h // 2), 5, (255, 0, 0), -1) # 프레임 중심점 표시

        # 이미지 출력
        cv2.imshow('result', img)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

    # 웹캠 해제 및 모든 창 닫기
    cap.release()
    cv2.destroyAllWindows()


if __name__=="__main__":
    main()
