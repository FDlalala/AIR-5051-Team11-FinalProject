#!/usr/bin/env python3
import cv2
from vision_module import VisionModule, pixel_to_world

def live_color_tracking(color="red"):
    vision = VisionModule()
    print(f"[视觉模块] 实时检测颜色：{color}（按 q 键退出）")

    while True:
        ret, frame = vision.cap.read()
        if not ret:
            print("无法读取摄像头图像")
            break

        pixel = vision.detect_color_once(color)
        if pixel:
            cx, cy = pixel
            xw, yw, zw = pixel_to_world(cx, cy)

            # 在画面上标记检测中心和世界坐标
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            label = f"({cx},{cy}) -> ({xw:.3f},{yw:.3f},{zw:.3f})"
            cv2.putText(frame, label, (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 1)

        cv2.imshow("Live Color Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    vision.release()

if __name__ == "__main__":
    live_color_tracking("red")  # 可改为 "green" 或 "blue"
