#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
# 添加脚本目录到Python路径，确保可以导入其他模块
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rospy
import time
import numpy as np
from voice_module import recognize_from_wav
from pick_demo import PickDemo
from geometry_msgs.msg import Pose

class VoiceScanRobot:
    def __init__(self):
        # 初始化语音控制机器人
        print("初始化语音扫描机器人系统...")
        
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
        
        # 扫描姿态 - 从上往下看，参照calibration_pose.py
        self.scan_pose = {
            "pitch": 1.57,  # 90度，垂直向下
            "z_height": 0.15  # 扫描高度
        }
        
        # 扫描路径配置 - 从绿色位置到蓝色位置
        self.scan_path = self.generate_scan_path()
        
        print("系统初始化完成")
    
    def generate_scan_path(self):
        """生成扫描路径 - 从绿色位置附近到蓝色位置"""
        # 获取绿色和蓝色的位置
        green_pos = self.robot.workspace["绿"]["position"]
        blue_pos = self.robot.workspace["蓝"]["position"]
        
        # 计算起点和终点 (降低x值一些，以便有更好的视野)
        start_x = green_pos[0] - 0.03
        start_y = green_pos[1]
        end_x = blue_pos[0] - 0.03
        end_y = blue_pos[1]
        
        # 生成路径点 - 创建5个点
        num_points = 5
        x_values = np.linspace(start_x, end_x, num_points)
        y_values = np.linspace(start_y, end_y, num_points)
        
        path = []
        for i in range(num_points):
            path.append((x_values[i], y_values[i]))
        
        print(f"生成扫描路径: {path}")
        return path
    
    def perform_scan(self, target_color):
        """执行扫描动作寻找目标"""
        print(f"\n开始扫描寻找 {target_color} 色物体...")
        
        # 先移动到安全高度
        self.robot.go_home()
        
        # 设置扫描姿态
        scan_z = self.scan_pose["z_height"]
        scan_pitch = self.scan_pose["pitch"]
        
        # 根据pitch角度计算四元数
        qx = 0.0
        qy = np.sin(scan_pitch/2)
        qz = 0.0
        qw = np.cos(scan_pitch/2)
        
        # 执行扫描路径
        for i, (x, y) in enumerate(self.scan_path):
            print(f"扫描位置 {i+1}/{len(self.scan_path)}: ({x:.3f}, {y:.3f}, {scan_z:.3f})")
            
            # 移动到扫描位置
            success = self.robot.move_to(x, y, scan_z, qx, qy, qz, qw)
            if not success:
                print(f"移动到扫描位置 {i+1} 失败")
                continue
            
            # 在每个位置暂停一会，模拟检测
            rospy.sleep(1.0)
            
            # 如果是目标颜色对应的位置，则模拟找到目标
            found = False
            if target_color == "红" and i == 2:  # 中间位置
                found = True
            elif target_color == "绿" and i == 0:  # 第一个位置
                found = True
            elif target_color == "蓝" and i == 4:  # 最后一个位置
                found = True
                
            if found:
                print(f"在扫描位置 {i+1} 找到 {target_color} 色物体！")
                return True
        
        print(f"扫描完成，未找到 {target_color} 色物体")
        return False
    
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
        print(f"识别到颜色: {color} -> 执行扫描寻找 {target_color} 色物体")
        
        # 4. 执行扫描寻找目标
        found = self.perform_scan(target_color)
        
        # 5. 如果找到目标，执行抓取任务
        if found:
            print(f"开始抓取 {target_color} 色物体")
            success = self.robot.pick_and_drop(target_color)
            
            if success:
                print(f"完成 {target_color} 色物体抓取任务")
            else:
                print(f"抓取 {target_color} 物体失败")
            
            return success
        else:
            print(f"未找到 {target_color} 色物体，取消抓取任务")
            return False
    
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
            # 每次处理完一个文件后暂停一段时间
            rospy.sleep(2.0)

def main():
    """主函数"""
    # 不需要在这里初始化ROS节点，因为PickDemo类已经初始化了
    
    controller = VoiceScanRobot()
    
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