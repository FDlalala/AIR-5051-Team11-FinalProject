import cv2
import mediapipe as mp
import os
# 抑制 TensorFlow C++ 日志 (0=all,1=INFO,2=WARNING,3=ERROR)
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
# 抑制 glog 日志 (0=INFO,1=WARNING,2=ERROR,3=FATAL)
os.environ['GLOG_minloglevel'] = '2'
import logging
# 抑制 absl 包的日志
logging.getLogger('absl').setLevel(logging.ERROR)

# MediaPipe Hands 模块初始化
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

def is_open_hand(landmarks: list) -> bool:
    """
    简单判断手势是否为“张开手”。
    利用 5 个指尖 (tip) 与对应的第二关节 (pip) 的 y 坐标关系：
    当 tip.y < pip.y 时，认为该指已伸直。
    如果 5 根手指都伸直，则返回 True，否则 False。
    """
    tip_ids = [4, 8, 12, 16, 20]
    pip_ids = [2, 6, 10, 14, 18]
    for tip, pip in zip(tip_ids, pip_ids):
        if landmarks[tip].y >= landmarks[pip].y:
            return False
    return True

def main():
    # 打开默认摄像头（设备号 0）
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    # Hands 模型参数可调：检测置信度、追踪置信度
    with mp_hands.Hands(
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5
    ) as hands:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # 翻转镜像，更符合习惯
            frame = cv2.flip(frame, 1)

            # 转为 RGB 供 MediaPipe 处理
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img_rgb.flags.writeable = False
            results = hands.process(img_rgb)
            img_rgb.flags.writeable = True
            annotated = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

            gesture_text = "No Hand"

            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]
                mp_drawing.draw_landmarks(
                    annotated,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing.DrawingSpec(thickness=2, circle_radius=2),
                    mp_drawing.DrawingSpec(thickness=2)
                )
                # 手势判断
                if is_open_hand(hand_landmarks.landmark):
                    gesture_text = "Open Hand"
                else:
                    gesture_text = "Fist"
                # 控制台打印
                print(f"Detected Gesture: {gesture_text}")

            # 在画面上标注手势
            cv2.putText(
                annotated,
                gesture_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 0, 0),
                2,
                cv2.LINE_AA
            )

            cv2.imshow("Hand Gesture Recognition", annotated)

            # 按下 ESC 键退出
            if cv2.waitKey(1) & 0xFF == 27:
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()