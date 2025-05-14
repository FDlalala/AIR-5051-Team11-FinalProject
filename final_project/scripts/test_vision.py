#!/usr/bin/env python3
from vision_module import VisionModule, pixel_to_world

def test_color(color):
    print(f"开始检测颜色：{color}")
    vision = VisionModule()
    pos = vision.get_stable_pixel_position(color)
    vision.release()

    if pos:
        print(f"像素坐标：{pos}")
        world = pixel_to_world(*pos)
        print(f"转换后世界坐标：{world}")
    else:
        print("未检测到稳定颜色目标")

if __name__ == "__main__":
    test_color("red")  # 可改为 green / blue
