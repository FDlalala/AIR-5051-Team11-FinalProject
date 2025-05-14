# gesture_module.py

import cv2
import mediapipe as mp
import threading
import time

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

class GestureRecognizer:
    def __init__(self, callback=None):
        self.callback = callback
        self.running = False
        self.hand_detector = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.5)

    def is_open_hand(self, landmarks):
        fingers = [8, 12, 16, 20]
        for i in fingers:
            if landmarks[i].y >= landmarks[i - 2].y:
                return False
        return True

    def detect_loop(self):
        self.running = True
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("[手势识别] 未检测到摄像头，跳过手势识别模块")
            return  # 或 raise Exception("Camera not available")

        prev_gesture = None

        while self.running:
            ret, frame = cap.read()
            if not ret:
                continue
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hand_detector.process(frame_rgb)

            gesture = None
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                    if self.is_open_hand(hand_landmarks.landmark):
                        gesture = "open"
                    else:
                        gesture = "fist"
                    break  # 只取一只手的结果

            # 只在状态改变时回调
            if gesture != prev_gesture:
                prev_gesture = gesture
                if self.callback:
                    if gesture == "open":
                        self.callback("open gripper")
                    elif gesture == "fist":
                        self.callback("close gripper")

            # 可选：展示窗口用于调试
            cv2.putText(frame, f"Gesture: {gesture or 'None'}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Gesture Recognition", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        cap.release()
        cv2.destroyAllWindows()

    def stop(self):
        self.running = False

# 单独调试使用
if __name__ == '__main__':
    def demo_callback(command):
        print(f"[Gesture Detected] → {command}")

    recognizer = GestureRecognizer(callback=demo_callback)
    t = threading.Thread(target=recognizer.detect_loop)
    t.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        recognizer.stop()
        t.join()
