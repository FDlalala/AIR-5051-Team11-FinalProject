import numpy as np
import yaml
import os

# 默认参数文件路径
CONFIG_PATH = "/home/robotics/sagittarius_ws/src/final_project/config/vision_config.yaml"

# 加载标定参数
def load_calibration(config_path=CONFIG_PATH):
    with open(config_path, "r") as f:
        data = yaml.safe_load(f)
    A = np.array(data["regression"]["A"])
    B = np.array(data["regression"]["B"])
    z = data["regression"].get("default_z", 0.02)
    return A, B, z

# 像素坐标 → 世界坐标
def pixel_to_world(px, py):
    A, B, z = load_calibration()
    pixel_vec = np.array([[px], [py]])  # 2x1
    world_vec = A @ pixel_vec + np.array(B).reshape((2, 1))
    x, y = world_vec.flatten()
    return float(x), float(y), float(z)
