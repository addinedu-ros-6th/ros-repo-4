import cv2
import pickle
import mediapipe as mp

# landmark visibility 임계값 설정
VISIBILITY_THRESHOLD = 0.6
INITIAL_BIAS_LIMIT = 60
INITIAL_BODY_SIZE = 4000
MIN_BODY_SIZE = 1500
MAX_FAILED_COUNTER = 20

class BodyFollower():
    def __init__(self):
        # Mediapipe Pose  모듈 초기화
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_pose = mp.solutions.pose
        self.mp_hands = mp.solutions.hands

        self.sub_mode = 0  # 0: stop & searching 1: too far 2: move to left 3: move to right 4: following 
        self.failed_counter = 0

        self.img = None
        self.body_center = [0, 0]
        self.body_size = 0

        self.cur_center = [0, 0]
        self.cur_body_size = 0
    
    def run(self, image):
        # pose
        with self.mp_pose.Pose(min_detection_confidence=0.7, min_tracking_confidence=0.7) as pose:
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
            results = pose.process(image)            
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
                        # cv2.putText(image, f"{idx}", (x, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0))
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
                        self.img = cv2.putText(image, "Please move to left side.", (16, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
                    elif self.body_center[0] - (center_x + INITIAL_BIAS_LIMIT) > 0:
                        self.sub_mode = 3
                        self.img = cv2.putText(image, "Please move to right side.", (16, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
                    else:
                        self.img = self.sub_mode = 4
                        cv2.putText(image, "I recognized you! Let's go!", (16, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
                # 화면 내 상체 크기가 일정 이하인 경우
                else:
                    self.sub_mode = 1
                    self.img = cv2.putText(image, "Please come a little closer.", (16, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
            
            # ---- 주행 단계 (인식 후 following or guiding) ----
            elif self.sub_mode == 4:
                if self.failed_counter == 0:
                    self.body_center = self.cur_center
                    self.body_size = self.cur_body_size
                    self.img = cv2.putText(image, "Following...", (16, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
                elif self.failed_counter <= MAX_FAILED_COUNTER:
                    self.img = cv2.putText(self.img, "Following...", (16, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
                else:                    
                    self.reset(image)
            # ----------------------------------------------
            
            diff_x = center_x - self.body_center[0]
            diff_y = center_y - self.body_center[1]
            
            # 상체 무게 중심 좌표 표시                
            text = f"mode: {self.sub_mode}, diff_x: {diff_x}, diff_y: {diff_y}, body_size: {self.body_size}, counter : {self.failed_counter}" 
            send_img = cv2.circle(self.img, (self.body_center[0], self.body_center[1]), 7, (0, 0, 255), -1)
            send_img = cv2.putText(send_img, text, (32, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)

            return send_img, self.sub_mode, diff_x, diff_y, self.body_size

        
    def reset(self, image):
        self.sub_mode = 0  
        self.failed_counter = 0

        self.img = image
        self.body_center = [0, 0]
        self.body_size = 0

        self.cur_center = [0, 0]
        self.cur_body_size = 0
            