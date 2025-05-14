#!/usr/bin/env python3
import rospy
import moveit_commander
import geometry_msgs.msg

class RobotController:
    def __init__(self):
        moveit_commander.roscpp_initialize([])
        # rospy.init_node('robot_controller', anonymous=True)

        # 命名空间设置
        ns = "sgr532"
        robot_description_ns = f"{ns}/robot_description"

        # 初始化控制组
        self.arm_group = moveit_commander.MoveGroupCommander(
            "sagittarius_arm", ns=ns, robot_description=robot_description_ns
        )
        self.gripper_group = moveit_commander.MoveGroupCommander(
            "sagittarius_gripper", ns=ns, robot_description=robot_description_ns
        )

        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.5)

    def open_gripper(self):
        self.gripper_group.set_named_target("open")
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()

    def close_gripper(self):
        self.gripper_group.set_named_target("close")
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()


    def move_to(self, x, y, z):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.w = 1.0  # 默认方向

        self.arm_group.set_pose_target(pose_goal)
        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return plan

    def go_home(self):
        self.arm_group.set_named_target("home")
        self.arm_group.go(wait=True)
        self.arm_group.stop()

if __name__ == '__main__':
    controller = RobotController()
    controller.go_home()
    rospy.sleep(5)
    controller.open_gripper()
    rospy.sleep(5)
    controller.close_gripper()
    rospy.sleep(5)
    controller.move_to(0.3, 0.0, 0.2)
