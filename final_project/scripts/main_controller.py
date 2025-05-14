#!/usr/bin/env python3
import sys
import os
# 添加脚本目录到Python路径，确保可以导入其他模块
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rospy
import time
from robot_controller import RobotController
from voice_module import recognize_from_wav

class MainController:
    def __init__(self):
        rospy.init_node('sagittarius_main_controller', anonymous=True)
        self.robot = RobotController()
        
        # 颜色映射
        self.color_map = {
            "红": "red",
            "绿": "green", 
            "蓝": "blue"
        }
        
        # 模拟各颜色物体的固定坐标
        self.fixed_positions = {
            "red": (0.25, 0.1, 0.02),    # 红色物体固定位置
            "green": (0.2, 0.0, 0.02),   # 绿色物体固定位置
            "blue": (0.25, -0.1, 0.02)   # 蓝色物体固定位置
        }

        print("系统初始化完成，等待语音指令...")
    
    def run(self, wav_path=None):
        if wav_path is None:
            # 默认使用红色中文语音文件
            wav_path = "/home/robotics/sagittarius_ws/src/final_project/test_data/colour/红色中文.wav"
        
        # 机器人归位
        print("机器人归位中...")
        self.robot.go_home()
        
        # 识别指令
        print(f"正在识别语音指令：{wav_path}")
        color_zh, code = recognize_from_wav(wav_path)
        
        if color_zh not in self.color_map:
            print(f"无法识别的颜色指令: {color_zh}")
            return False
        
        color_en = self.color_map[color_zh]
        print(f"识别到颜色指令: {color_zh}({color_en})")
        
        # 打开夹爪
        self.robot.open_gripper()
        print("夹爪已打开")
        
        # 使用固定坐标代替摄像头检测
        print(f"模拟检测{color_zh}色物体位置...")
        if color_en in self.fixed_positions:
            x, y, z = self.fixed_positions[color_en]
            print(f"目标位置 - 世界坐标: ({x:.3f}, {y:.3f}, {z:.3f})")
        else:
            print(f"错误：未设置{color_zh}色物体的固定坐标")
            return False
        
        # 移动到目标上方
        z_approach = z + 0.05  # 先到物体上方5cm处
        print(f"移动到目标上方: ({x:.3f}, {y:.3f}, {z_approach:.3f})")
        self.robot.move_to(x, y, z_approach)
        rospy.sleep(1)
        
        # 下降到抓取高度
        print(f"下降到抓取高度: ({x:.3f}, {y:.3f}, {z:.3f})")
        self.robot.move_to(x, y, z)
        rospy.sleep(1)
        
        # 关闭夹爪抓取
        print("关闭夹爪抓取物体")
        self.robot.close_gripper()
        rospy.sleep(1)
        
        # 抓取后抬高
        print("抬高物体")
        self.robot.move_to(x, y, z_approach)
        rospy.sleep(1)
        
        # 回到初始位置
        print("回到初始位置")
        self.robot.go_home()
        
        print("任务完成")
        return True
    
    def manual_test(self, color_en="red"):
        """
        手动测试特定颜色的抓取，不使用语音识别
        """
        print(f"手动测试抓取{color_en}色物体...")
        
        # 打开夹爪
        self.robot.go_home()
        self.robot.open_gripper()
        print("夹爪已打开")
        
        # 获取固定坐标
        if color_en in self.fixed_positions:
            x, y, z = self.fixed_positions[color_en]
            print(f"目标位置 - 世界坐标: ({x:.3f}, {y:.3f}, {z:.3f})")
    else:
            print(f"错误：未设置{color_en}色物体的固定坐标")
            return False
        
        # 执行抓取流程
        z_approach = z + 0.05
        print(f"移动到目标上方: ({x:.3f}, {y:.3f}, {z_approach:.3f})")
        self.robot.move_to(x, y, z_approach)
        rospy.sleep(1)
        
        print(f"下降到抓取高度: ({x:.3f}, {y:.3f}, {z:.3f})")
        self.robot.move_to(x, y, z)
        rospy.sleep(1)
        
        print("关闭夹爪抓取物体")
        self.robot.close_gripper()
        rospy.sleep(1)
        
        print("抬高物体")
        self.robot.move_to(x, y, z_approach)
        rospy.sleep(1)
        
        print("回到初始位置")
        self.robot.go_home()
        
        print("手动测试完成")
        return True

if __name__ == '__main__':
    try:
        controller = MainController()
        
        # 测试语音文件
        test_files = {
            "红色": "/home/robotics/sagittarius_ws/src/final_project/test_data/colour/红色中文.wav",
            "绿色": "/home/robotics/sagittarius_ws/src/final_project/test_data/colour/绿色中文.wav",
            "蓝色": "/home/robotics/sagittarius_ws/src/final_project/test_data/colour/蓝色中文.wav"
        }
        
        # 使用方式选择
        use_voice = False  # 设置为False则使用手动测试，不需要语音识别
        
        if use_voice:
            # 通过语音文件识别
            color_to_test = "红色"  # 可改为 "绿色" 或 "蓝色"
            wav_file = test_files[color_to_test]
            controller.run(wav_file)
        else:
            # 手动测试特定颜色，不使用语音识别
            target_color = "red"  # 可改为 "green" 或 "blue"
            controller.manual_test(target_color)
        
    except rospy.ROSInterruptException:
        print("程序被中断")
    except Exception as e:
        print(f"发生错误: {e}")
