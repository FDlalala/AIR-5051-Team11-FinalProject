import cv2
import numpy as np
import time
import yaml
import os

# 全局加载一次标定参数
vision_config_path = "/home/robotics/sagittarius_ws/src/final_project/config/vision_config.yaml"

def load_calibration():
    with open(vision_config_path, 'r') as f:
        config = yaml.safe_load(f)

    lin = config["LinearRegression"]
    k1 = lin["k1"]
    k2 = lin["k2"]
    b1 = lin["b1"]
    b2 = lin["b2"]
    z = 0.015  # 固定高度值，可根据实验微调
    return k1, k2, b1, b2, z

k1, k2, b1, b2, z_fixed = load_calibration()

def pixel_to_world(x_pixel, y_pixel):
    x = k1 * x_pixel + b1
    y = k2 * y_pixel + b2
    return float(x), float(y), float(z_fixed)

def track_stable_position(color, max_attempts=30, window=5, threshold=10):
    """
    连续检测颜色目标，直到位置稳定为止。
    - window: 多少帧用于判断稳定
    - threshold: 最大像素波动范围
    """
    positions = []

    for _ in range(max_attempts):
        cx, cy = get_object_pixel_position(color)
        if cx is not None and cy is not None:
            positions.append((cx, cy))
            if len(positions) > window:
                positions.pop(0)

            if len(positions) == window:
                xs, ys = zip(*positions)
                if (max(xs) - min(xs) < threshold) and (max(ys) - min(ys) < threshold):
                    mean_x = int(sum(xs) / window)
                    mean_y = int(sum(ys) / window)
                    return mean_x, mean_y

        time.sleep(0.1)

    return None

class VisionModule:
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError("无法打开摄像头")

        # HSV颜色范围
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

    def detect_color_once(self, color):
        ret, frame = self.cap.read()
        if not ret:
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        params = self.hsv_ranges.get(color)
        if params is None:
            return None

        lower = np.array([params["h_min"], params["s_min"], params["v_min"]])
        upper = np.array([params["h_max"], params["s_max"], params["v_max"]])
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            if cv2.contourArea(c) < 500:
                continue
            x, y, w, h = cv2.boundingRect(c)
            cx, cy = x + w // 2, y + h // 2
            return (cx, cy)

        return None

    def get_stable_pixel_position(self, color, timeout=5.0, buffer_size=10, tolerance=10):
        positions = []
        start_time = time.time()

        while time.time() - start_time < timeout:
            pos = self.detect_color_once(color)
            if pos is not None:
                positions.append(pos)
                if len(positions) > buffer_size:
                    positions.pop(0)

                if len(positions) == buffer_size:
                    xs, ys = zip(*positions)
                    if max(xs) - min(xs) < tolerance and max(ys) - min(ys) < tolerance:
                        avg_x = int(sum(xs) / buffer_size)
                        avg_y = int(sum(ys) / buffer_size)
                        return (avg_x, avg_y)

            time.sleep(0.1)

        return None

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()
