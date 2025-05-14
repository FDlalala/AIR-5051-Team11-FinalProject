#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
# 添加脚本目录到Python路径，确保可以导入其他模块
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time
from copy import deepcopy

class PickDemo:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('sagittarius_pick_demo', anonymous=True)
        
        # 初始化MoveIt
        moveit_commander.roscpp_initialize([])
        
        # 命名空间设置
        ns = "sgr532"
        robot_description_ns = f"{ns}/robot_description"
        
        # 初始化机械臂和夹爪组
        self.arm_group = moveit_commander.MoveGroupCommander(
            "sagittarius_arm", ns=ns, robot_description=robot_description_ns
        )
        self.gripper_group = moveit_commander.MoveGroupCommander(
            "sagittarius_gripper", ns=ns, robot_description=robot_description_ns
        )
        
        # 设置运动规划参数
        self.arm_group.set_max_velocity_scaling_factor(0.3)  # 降低速度确保安全
        self.arm_group.set_max_acceleration_scaling_factor(0.2)  # 降低加速度确保安全
        self.arm_group.set_goal_position_tolerance(0.001)  # 提高位置精度
        self.arm_group.set_goal_orientation_tolerance(0.01)  # 提高姿态精度
        
        # 允许重新规划
        self.arm_group.allow_replanning(True)
        
        # 设置默认规划时间，具体颜色会在方法中单独设置
        self.arm_group.set_planning_time(10.0)  # 默认规划时间
        
        # 获取终端链接名称
        self.end_effector_link = self.arm_group.get_end_effector_link()
        
        # 根据实测坐标定义工作空间内的位置
        # 蓝色位置是实际测量的抓取点：0.31353 -0.11674 0.037976
        # 红色位置在中间
        # 绿色位置在y轴对称位置
        self.workspace = {
            "蓝": {
                "position": [0.31353, -0.11674, 0.037976],  # 蓝色物体位置（已知）
                "approach_height": 0.13,                    # 接近高度（比抓取高度高约10cm）
                "grasp_height": 0.037976,                   # 抓取高度（与物体一致）
                "orientation": [0.0081098, 0.016793, -0.019634, 0.99963]  # 抓取姿态（四元数）
            },
            "红": {
                "position": [0.31353, 0.0, 0.037976],       # 红色物体位置（中间）
                "approach_height": 0.13,                    # 接近高度
                "grasp_height": 0.037976,                   # 抓取高度
                "orientation": [0.0081098, 0.016793, -0.019634, 0.99963]  # 抓取姿态
            },
            "绿": {
                "position": [0.31353, 0.11674, 0.037976],   # 绿色物体位置（y对称）
                "approach_height": 0.13,                    # 接近高度
                "grasp_height": 0.037976,                   # 抓取高度
                "orientation": [0.0081098, 0.016793, -0.019634, 0.99963]  # 抓取姿态
            },
            "home": [0.0, 0.0, 0.25],                      # 初始位置
            "drop_point": {                                 # 物品丢弃点位置
                "position": [0.011441, 0.20021, 0.27945],
                "orientation": [-0.016169, 0.0077053, 0.70079, 0.71314]
            },
            # 添加中间点，用于分段规划到丢弃点
            "mid_point": {
                "position": [0.15, 0.0, 0.20],              # 中间安全点位置
                "orientation": [0.0, 0.0, 0.0, 1.0]         # 默认姿态
            }
        }
        
        # 颜色的规划时间设置
        self.color_planning_times = {
            "红": 10.0,   # 红色规划时间10秒
            "绿": 10.0,   # 绿色规划时间10秒
            "蓝": 50.0    # 蓝色规划时间50秒
        }
        
        print("系统初始化完成")
    
    def open_gripper(self):
        """打开夹爪"""
        print("打开夹爪")
        self.gripper_group.set_named_target("open")
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()
        rospy.sleep(0.5)  # 等待夹爪完全打开
    
    def close_gripper(self):
        """关闭夹爪"""
        print("关闭夹爪")
        self.gripper_group.set_named_target("close")
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()
        rospy.sleep(0.5)  # 等待夹爪完全闭合
    
    def go_home(self):
        """回到初始位置"""
        print("机械臂归位中...")
        self.arm_group.set_named_target("home")
        plan_success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        if not plan_success:
            print("警告：归位失败，尝试使用笛卡尔路径")
            self.move_to(*self.workspace["home"])
        
        rospy.sleep(1)
    
    def move_to(self, x, y, z, qx=None, qy=None, qz=None, qw=None):
        """移动到特定位置和姿态"""
        print(f"移动到位置: ({x:.3f}, {y:.3f}, {z:.3f})")
        
        pose_goal = Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        # 如果有提供姿态，则使用提供的姿态
        # 否则使用默认姿态（垂直向下）
        if qx is not None and qy is not None and qz is not None and qw is not None:
            pose_goal.orientation.x = qx
            pose_goal.orientation.y = qy
            pose_goal.orientation.z = qz
            pose_goal.orientation.w = qw
            print(f"使用指定姿态: ({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})")
        else:
            # 默认姿态：垂直向下
            pose_goal.orientation.x = 0.0
            pose_goal.orientation.y = 0.0
            pose_goal.orientation.z = 0.0
            pose_goal.orientation.w = 1.0
            print("使用默认姿态：垂直向下")
        
        self.arm_group.set_pose_target(pose_goal)
        
        # 尝试规划和执行
        plan_success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        rospy.sleep(0.5)  # 等待稳定
        return plan_success
    
    def move_cartesian(self, waypoints, eef_step=0.01, jump_threshold=0.0):
        """使用笛卡尔路径规划"""
        print(f"执行笛卡尔路径规划，包含 {len(waypoints)} 个路点")
        
        fraction = 0.0
        attempts = 0
        max_attempts = 5
        
        while fraction < 1.0 and attempts < max_attempts:
            (plan, fraction) = self.arm_group.compute_cartesian_path(
                waypoints,      # 路点列表
                eef_step,       # 终端步进值，每隔多少米计算一次逆解
                jump_threshold  # 跳跃阈值
            )
            attempts += 1
            print(f"笛卡尔路径规划尝试 {attempts}，覆盖率为 {fraction:.2f}")
        
        if fraction > 0.7:  # 至少要有70%的覆盖率
            print(f"路径规划成功，覆盖率: {fraction:.2f}，执行规划...")
            self.arm_group.execute(plan, wait=True)
            return True
        else:
            print(f"笛卡尔路径规划失败，覆盖率过低: {fraction:.2f}")
            return False
    
    def approach_vertical(self, x, y, grasp_z, approach_z, orientation):
        """直上直下接近物体的策略，确保避免碰撞"""
        # 1. 先移动到物体正上方的安全高度
        print(f"直上直下策略: 先移动到安全高度 ({x:.3f}, {y:.3f}, {approach_z:.3f})")
        qx, qy, qz, qw = orientation
        
        # 移动到接近位置（物体正上方）
        if not self.move_to(x, y, approach_z, qx, qy, qz, qw):
            print("移动到接近位置失败")
            return False
            
        # 2. 缓慢下降到抓取高度
        print(f"直上直下策略: 垂直下降到抓取高度 ({x:.3f}, {y:.3f}, {grasp_z:.3f})")
        if not self.move_to(x, y, grasp_z, qx, qy, qz, qw):
            print("垂直下降到抓取高度失败")
            return False
            
        return True
    
    def go_to_drop_point_multi_step(self):
        """分段移动到丢弃点，避免规划失败"""
        print("使用多段规划策略移动到丢弃点...")
        
        # 1. 先移动到安全高度
        current_pose = self.arm_group.get_current_pose().pose
        if not self.move_to(current_pose.position.x, current_pose.position.y, 0.20):
            print("移动到安全高度失败")
            return False
        
        # 2. 移动到中间点
        mid_point = self.workspace["mid_point"]
        if not self.move_to(*mid_point["position"]):
            print("移动到中间点失败")
            return False
        
        # 3. 最后移动到丢弃点
        drop_point = self.workspace["drop_point"]
        x, y, z = drop_point["position"]
        qx, qy, qz, qw = drop_point["orientation"]
        
        if not self.move_to(x, y, z, qx, qy, qz, qw):
            print("从中间点移动到丢弃点失败")
            return False
        
        return True
    
    def go_to_drop_point_cartesian(self):
        """使用笛卡尔路径规划移动到丢弃点"""
        print("使用笛卡尔路径规划移动到丢弃点...")
        
        # 获取当前位姿
        start_pose = self.arm_group.get_current_pose().pose
        
        # 创建路点列表
        waypoints = []
        waypoints.append(deepcopy(start_pose))
        
        # 添加中间点 - 先抬高
        mid_pose = deepcopy(start_pose)
        mid_pose.position.z = 0.20
        waypoints.append(deepcopy(mid_pose))
        
        # 添加丢弃点
        drop_point = self.workspace["drop_point"]
        end_pose = deepcopy(start_pose)
        end_pose.position.x = drop_point["position"][0]
        end_pose.position.y = drop_point["position"][1]
        end_pose.position.z = drop_point["position"][2]
        end_pose.orientation.x = drop_point["orientation"][0]
        end_pose.orientation.y = drop_point["orientation"][1]
        end_pose.orientation.z = drop_point["orientation"][2]
        end_pose.orientation.w = drop_point["orientation"][3]
        waypoints.append(end_pose)
        
        # 执行笛卡尔路径规划
        return self.move_cartesian(waypoints)
    
    def go_to_drop_point(self):
        """移动到物品丢弃点，尝试多种规划方式"""
        print("移动到物品丢弃点...")
        
        drop_data = self.workspace["drop_point"]
        x, y, z = drop_data["position"]
        qx, qy, qz, qw = drop_data["orientation"]
        
        # 策略1: 直接移动到丢弃点
        print("策略1: 直接移动到丢弃点")
        if self.move_to(x, y, z, qx, qy, qz, qw):
            print("直接移动到丢弃点成功")
            return True
        
        # 策略2: 笛卡尔路径规划
        print("策略1失败，尝试策略2: 笛卡尔路径规划")
        if self.go_to_drop_point_cartesian():
            print("使用笛卡尔路径规划到丢弃点成功")
            return True
        
        # 策略3: 多段规划
        print("策略2失败，尝试策略3: 多段规划")
        if self.go_to_drop_point_multi_step():
            print("使用多段规划到丢弃点成功")
            return True
        
        # 策略4: 尝试使用关节空间规划
        print("策略3失败，尝试策略4: 关节空间规划")
        
        # 保存当前位置用于恢复
        current_pose = self.arm_group.get_current_pose().pose
        
        # 尝试先移动到安全位置，再规划
        print("先移动到安全位置")
        try:
            # 记录当前关节值
            current_joints = self.arm_group.get_current_joint_values()
            
            # 设置特定关节目标
            joint_goal = current_joints.copy()
            
            # 根据丢弃点的位置，设置关节角度（需要机器人调试确定）
            # 这里只是示例，实际值需要根据具体机器人调整
            joint_goal[0] = 0.1  # 例如，调整第一个关节的值
            joint_goal[1] = 0.2  # 例如，调整第二个关节的值
            
            self.arm_group.go(joint_goal, wait=True)
            self.arm_group.stop()
            
            # 再次尝试直接规划到丢弃点
            if self.move_to(x, y, z, qx, qy, qz, qw):
                print("先调整关节角度后，成功移动到丢弃点")
                return True
            
        except Exception as e:
            print(f"关节空间规划出错: {e}")
            
            # 恢复到原始位置
            try:
                self.arm_group.set_pose_target(current_pose)
                self.arm_group.go(wait=True)
                self.arm_group.stop()
            except:
                pass
        
        print("所有规划策略都失败了，无法移动到丢弃点")
        return False
    
    def pick_and_drop(self, color):
        """完整的抓取-丢弃流程：抓取物体并丢弃到指定位置"""
        if color not in self.workspace:
            print(f"错误：未找到颜色 {color} 的位置数据")
            return False
        
        print(f"\n开始抓取并丢弃 {color} 色物体")
        
        # 根据颜色设置不同的规划时间
        if color in self.color_planning_times:
            planning_time = self.color_planning_times[color]
            print(f"为 {color} 色物体设置规划时间: {planning_time}秒")
            self.arm_group.set_planning_time(planning_time)
        
        # 1. 机械臂归位，夹爪打开
        self.go_home()
        self.open_gripper()
        
        # 2. 获取物体位置和高度信息
        obj_data = self.workspace[color]
        x, y, z = obj_data["position"]
        approach_z = obj_data["approach_height"]
        grasp_z = obj_data["grasp_height"]
        orientation = obj_data["orientation"]
        
        # 3. 使用直上直下策略接近物体
        print(f"接近 {color} 物体...")
        if not self.approach_vertical(x, y, grasp_z, approach_z, orientation):
            print("接近物体失败")
            self.go_home()
            return False
        
        # 4. 关闭夹爪抓取物体
        self.close_gripper()
        
        # 5. 垂直提起物体到安全高度
        print("垂直提起物体")
        qx, qy, qz, qw = orientation
        if not self.move_to(x, y, approach_z, qx, qy, qz, qw):
            print("提起物体失败")
            self.open_gripper()  # 释放物体
            self.go_home()
            return False
        
        # 特殊处理蓝色物体 - 添加中间过渡点
        if color == "蓝":
            print("检测到蓝色物体，添加额外中间过渡点...")
            
            # 先移动到安全高度
            if not self.move_to(x, y, 0.20):
                print("移动到安全高度失败")
                self.open_gripper()
                self.go_home()
                return False
            
            # 移动到红色物体位置上方（作为中间点）
            red_data = self.workspace["红"]
            red_x, red_y, _ = red_data["position"]
            print(f"移动到红色位置上方作为中间点: ({red_x:.3f}, {red_y:.3f}, 0.20)")
            
            if not self.move_to(red_x, red_y, 0.20):
                print("移动到红色上方中间点失败")
                self.open_gripper()
                self.go_home()
                return False
            
            print("成功移动到中间过渡点")
        
        # 6. 移动到丢弃点
        print("带着物体移动到丢弃点")
        if not self.go_to_drop_point():
            print("移动到丢弃点失败，尝试返回初始位置")
            self.go_home()
            self.open_gripper()
            return False
        
        # 7. 在丢弃点释放物体
        print(f"丢弃 {color} 色物体")
        self.open_gripper()
        rospy.sleep(1)  # 等待物体完全掉落
        
        # 8. 回到初始位置
        print("返回初始位置")
        self.go_home()
        
        print(f"{color} 色物体抓取并丢弃完成")
        return True
    
    def pick_object(self, color):
        """执行抓取序列（不包含丢弃）"""
        if color not in self.workspace:
            print(f"错误：未找到颜色 {color} 的位置数据")
            return False
        
        print(f"\n开始抓取 {color} 色物体")
        
        # 根据颜色设置不同的规划时间
        if color in self.color_planning_times:
            planning_time = self.color_planning_times[color]
            print(f"为 {color} 色物体设置规划时间: {planning_time}秒")
            self.arm_group.set_planning_time(planning_time)
        
        # 1. 机械臂归位，夹爪打开
        self.go_home()
        self.open_gripper()
        
        # 2. 获取物体位置和高度信息
        obj_data = self.workspace[color]
        x, y, z = obj_data["position"]
        approach_z = obj_data["approach_height"]
        grasp_z = obj_data["grasp_height"]
        orientation = obj_data["orientation"]
        
        # 3. 使用直上直下策略接近物体
        print(f"接近 {color} 物体...")
        if not self.approach_vertical(x, y, grasp_z, approach_z, orientation):
            print("接近物体失败")
            self.go_home()
            return False
        
        # 4. 关闭夹爪抓取物体
        self.close_gripper()
        
        # 5. 垂直提起物体到安全高度
        print("垂直提起物体")
        qx, qy, qz, qw = orientation
        if not self.move_to(x, y, approach_z, qx, qy, qz, qw):
            print("提起物体失败")
            self.open_gripper()  # 释放物体
            self.go_home()
            return False
        
        # 6. 回到初始位置
        print("返回初始位置")
        self.go_home()
        
        print(f"{color} 色物体抓取完成")
        return True
    
    def release_object(self):
        """释放物体"""
        self.open_gripper()
    
    def run_demo(self):
        """运行完整演示"""
        try:
            # 依次抓取三种颜色的物体并丢弃
            for color in ["蓝", "红", "绿"]:
                success = self.pick_and_drop(color)
                if not success:
                    print(f"抓取并丢弃 {color} 物体失败，跳过")
                
                # 短暂暂停，准备下一个
                rospy.sleep(1)
            
            print("\n演示完成")
        except rospy.ROSInterruptException:
            print("程序被中断")
        except Exception as e:
            print(f"发生错误: {e}")
        finally:
            # 确保最后回到安全位置
            self.go_home()
            self.open_gripper()

if __name__ == "__main__":
    try:
        # 创建演示对象
        demo = PickDemo()
        
        # 可以选择运行完整演示，抓取并丢弃特定颜色，或只抓取不丢弃
        run_full_demo = False           # 设置为True将运行完整演示
        include_drop = True             # 设置为True将包含丢弃动作
        target_color = "蓝"              # 指定要抓取的颜色
        
        if run_full_demo:
            demo.run_demo()
        else:
            if include_drop:
                # 抓取并丢弃
                demo.pick_and_drop(target_color)
            else:
                # 只抓取不丢弃
                demo.pick_object(target_color)
                rospy.sleep(2)  # 展示2秒
                demo.release_object()
                demo.go_home()
            
    except rospy.ROSInterruptException:
        print("程序被中断")
    except Exception as e:
        print(f"发生错误: {e}") 