#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
# 添加脚本目录到Python路径，确保可以导入其他模块
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rospy
import time
from voice_module import recognize_from_wav
from pick_demo import PickDemo

class VoiceControlRobot:
    def __init__(self):
        # 初始化语音控制机器人
        print("初始化语音控制机器人系统...")
        
        # 语音测试目录
        self.voice_dir = "/home/robotics/sagittarius_ws/src/final_project/test_data/colour"
        
        # 颜色映射关系 - 从语音识别结果映射到机械臂控制颜色
        self.color_mapping = {
            "红": "红",
            "绿": "绿",
            "蓝": "蓝"
        }
        
        # 初始化机械臂控制对象
        self.robot = PickDemo()
        
        print("系统初始化完成")
    
    def process_voice_command(self, wav_file):
        """处理语音命令并执行对应操作"""
        print(f"\n处理语音命令: {os.path.basename(wav_file)}")
        
        # 1. 语音识别
        color, code = recognize_from_wav(wav_file)
        
        # 2. 判断颜色是否有效
        if color not in self.color_mapping:
            print(f"未识别到有效颜色命令: {color}")
            return False
        
        # 3. 映射到机械臂控制颜色
        target_color = self.color_mapping[color]
        print(f"识别到颜色: {color} -> 执行抓取 {target_color} 色物体")
        
        # 4. 执行抓取任务
        success = self.robot.pick_and_drop(target_color)
        
        if success:
            print(f"完成 {target_color} 色物体抓取任务")
        else:
            print(f"抓取 {target_color} 物体失败")
        
        return success
    
    def process_single_file(self, wav_file):
        """处理单个语音文件"""
        if not os.path.exists(wav_file):
            print(f"错误: 文件 {wav_file} 不存在")
            return False
            
        return self.process_voice_command(wav_file)
    
    def process_all_files(self):
        """处理目录中的所有语音文件"""
        wav_files = [f for f in sorted(os.listdir(self.voice_dir)) if f.endswith(".wav")]
        
        if not wav_files:
            print(f"错误: 目录 {self.voice_dir} 中没有找到wav文件")
            return
        
        print(f"找到 {len(wav_files)} 个语音文件，开始处理...")
        
        for fname in wav_files:
            wav_path = os.path.join(self.voice_dir, fname)
            self.process_voice_command(wav_path)

def main():
    """主函数"""
    # 不需要在这里初始化ROS节点，因为PickDemo类已经初始化了
    # rospy.init_node('voice_control_robot', anonymous=True)
    
    controller = VoiceControlRobot()
    
    # 解析命令行参数
    if len(sys.argv) > 1:
        # 处理指定的语音文件
        wav_file = sys.argv[1]
        controller.process_single_file(wav_file)
    else:
        # 处理目录中的所有语音文件
        controller.process_all_files()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("程序被中断")
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"发生错误: {e}") 