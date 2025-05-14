import cv2
import numpy as np

class VisionModule:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("无法打开摄像头")

        # HSV颜色范围（包括浅蓝色）
        self.hsv_ranges = {
            "red": {
                "h_min": 0, "h_max": 10,
                "s_min": 100, "s_max": 255,
                "v_min": 100, "v_max": 255
            },
            "green": {
                "h_min": 40, "h_max": 80,
                "s_min": 50, "s_max": 255,
                "v_min": 50, "v_max": 255
            },
            "blue": {
                "h_min": 90, "h_max": 130,
                "s_min": 30, "s_max": 255,
                "v_min": 50, "v_max": 255
            }
        }

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            for color, params in self.hsv_ranges.items():
                lower = np.array([params["h_min"], params["s_min"], params["v_min"]])
                upper = np.array([params["h_max"], params["s_max"], params["v_max"]])
                mask = cv2.inRange(hsv, lower, upper)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for c in contours:
                    if cv2.contourArea(c) < 500:
                        continue
                    x, y, w, h = cv2.boundingRect(c)
                    cx, cy = x + w // 2, y + h // 2
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, f"{color}: ({cx}, {cy})", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    print(f"检测到 {color}，中心坐标: ({cx}, {cy})")

            cv2.imshow("Color Detection", frame)
            key = cv2.waitKey(1)
            if key == 27:  # ESC 键
                break

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    vision = VisionModule()
    vision.run()
