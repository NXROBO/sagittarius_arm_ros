#!/usr/bin/env python
# -*- coding: utf-8 -*-

from copy import deepcopy
import math
import sys
import moveit_commander
from moveit_commander import MoveGroupCommander
import rospy
from sagittarius_object_color_detector.srv import *

global arm


class MoveItCartesianDemo:
    # global arm
    # global cartesian
    # global end_effector_link
    # global gripper1
    waypoints = []

    def __init__(self):
        # global arm
        # global cartesian
        # global end_effector_link
        # global gripper1
        #
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cartesian_demo', anonymous=True)

        # 是否需要使用笛卡尔空间的运动规划，获取参数，如果没有设置，则默认为True，即走直线
        self.cartesian = rospy.get_param('~cartesian', False)

        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm_group = MoveGroupCommander('arm')

        # 初始化需要使用move group控制的机械臂中的gripper group
        self.gripper = MoveGroupCommander('gripper')
        self.gripper1 = moveit_commander.MoveGroupCommander('gripper')
        # 当运动规划失败后，允许重新规划
        self.arm_group.allow_replanning(False)

        # 设置目标位置所使用的参考坐标系
        self.arm_group.set_pose_reference_frame('sagittarius_base_link')

        # 设置目标位置所使用的参考坐标系
        self.gripper.set_pose_reference_frame('sagittarius_base_link')

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm_group.set_goal_position_tolerance(0.001)
        self.arm_group.set_goal_orientation_tolerance(0.001)

        self.gripper1.set_goal_joint_tolerance(0.001)
        # 设置允许的最大速度和加速度
        self.arm_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_group.set_max_velocity_scaling_factor(0.5)

        # 获取终端link的名称
        self.end_effector_link = self.arm_group.get_end_effector_link()
        # 控制机械臂先回到初始化位置
        self.arm_group.set_named_target('home')
        self.arm_group.go()
        rospy.sleep(1)

    def getPose(self):
        return self.arm_group.get_current_pose(self.end_effector_link).pose

    # width 8mm~65mm
    def gripper_catch(self, width=40):
        # 0~0.0315
        # if (15 > width):
        #     width = 15
        # if (width > 68):
        #     width = 68
        # a = 23
        # b = 16.25
        # c = width / 2 + 3.18

        # angle = math.degrees(math.acos((a*a - b*b - c*c) / (-2*b*c)))
        # value = -0.0315 / 117.7 * angle
        # if(value < -0.0315):
        #     value = -0.0315
        # print(width)
        # print(angle)
        # print(value)
        # self.gripper1.set_joint_value_target([value, value])
        self.gripper1.set_joint_value_target([-0.0197, -0.0197])
        self.gripper1.go()
        rospy.sleep(2)

    def gripper_open(self):
        self.gripper1.set_joint_value_target([0.0, 0.0])
        self.gripper1.go()
        rospy.sleep(2)

    def waypoints_clear(self):
        del self.waypoints[:]

    def waypoints_add(self, px, py, pz, ox, oy, oz, ow):
        if len(self.waypoints) == 0:
            self.waypoints.append(
                deepcopy(self.arm_group.get_current_pose(self.end_effector_link).pose))
        pose = deepcopy(self.arm_group.get_current_pose(
            self.end_effector_link).pose)
        pose.position.x = px
        pose.position.y = py
        pose.position.z = pz
        pose.orientation.x = ox
        pose.orientation.y = oy
        pose.orientation.z = oz
        pose.orientation.w = ow
        self.waypoints.append(deepcopy(pose))

    def waypoints_run(self):
        fraction = 0.0  # 路径覆盖率
        maxtries = 100  # 最大尝试规划次数
        attempts = 0  # 已尝试规划次数
        # 设置当前位置为运动初始位置
        self.arm_group.set_start_state_to_current_state()

        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm_group.compute_cartesian_path(
                self.waypoints, 0.01, 0.0, True)
            attempts += 1
            # if attempts % 10 == 0
            if fraction == 1.0:
                self.arm_group.execute(plan)
                self.waypoints_clear()
        if fraction < 1.0:
            rospy.logwarn(
                "Descartes planning faild, fraction = %d" % (fraction))

    @staticmethod
    def eular2orientation(pitch, yaw, roll):
        return math.sin(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.cos(yaw/2)*math.sin(roll/2), \
            math.sin(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.sin(yaw/2)*math.sin(roll/2), \
            math.cos(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.cos(yaw/2)*math.sin(roll/2), \
            math.cos(pitch/2)*math.cos(yaw/2)*math.cos(roll/2) - \
            math.sin(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)

    def move2pose_eular(self, times, px, py, pz, pitch=0, yaw=0, roll=0, wait=True):
        wpose = deepcopy(self.arm_group.get_current_pose(
            self.end_effector_link).pose)
        wpose.position.x = px
        wpose.position.y = py
        wpose.position.z = pz
        wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w = self.eular2orientation(
            pitch, yaw, roll)
        self.arm_group.set_pose_target(wpose)  # 自由曲线
        plan = self.arm_group.plan()
        if (len(plan.joint_trajectory.points) == 0):
            rospy.logwarn("Planning faild")
            return False
        self.arm_group.execute(plan, wait=wait)
        rospy.sleep(times)
        return True

    def move2pose(self, times, px, py, pz, ox, oy, oz, ow, wait=True):
        wpose = deepcopy(self.arm_group.get_current_pose(
            self.end_effector_link).pose)
        wpose.position.x = px
        wpose.position.y = py
        wpose.position.z = pz
        wpose.orientation.x = ox
        wpose.orientation.y = oy
        wpose.orientation.z = oz
        wpose.orientation.w = ow
        self.arm_group.set_pose_target(wpose)  # 自由曲线
        plan = self.arm_group.plan()
        if (len(plan.joint_trajectory.points) == 0):
            rospy.logwarn("Planning faild")
            return False
        self.arm_group.execute(plan, wait=wait)
        rospy.sleep(times)
        return True

    def stop(self):
        self.arm_group.set_named_target('home')
        self.arm_group.go()
        rospy.sleep(1)
        self.arm_group.set_named_target('sleep')
        self.arm_group.go()
        rospy.sleep(1)
        self.gripper1.set_joint_value_target([0, 0])
        self.gripper1.go()
        rospy.sleep(0.5)


if __name__ == "__main__":
    follow_flag = False
    color_flag = "blue"
    try:
        if(sys.argv[1] == "blue"):
            color_flag = "blue"
        elif(sys.argv[1] == "red"):
            color_flag = "red"
        elif(sys.argv[1] == "green"):
            color_flag = "green"
        elif(sys.argv[1] == "all"):
            color_flag = "all"
        else:
            raise

        if(sys.argv[2] == "true"):
            follow_flag = True
        elif(sys.argv[2] == "false"):
            follow_flag = False
        else:
            raise
    except:
        print("")
        print("    usage: catch_color.py <blue|red|green|all> <follow:true|false>")
        print("")
        exit()

    # 初始化ROS节点
    arm = MoveItCartesianDemo()

    # 初始位置
    default_x = 0.15
    default_y = 0.0
    default_z = 0.21
    x = default_x
    y = default_y
    z = default_z
    arm.move2pose_eular(1, x, y, z, pitch=math.radians(90))
    while(not rospy.is_shutdown()):
        try:
            # 请求服务
            rospy.wait_for_service('/object_detect')

            detect_object_service = rospy.ServiceProxy(
                '/object_detect', DetectObjectSrv)
            # print(detect_object_service(DetectObjectSrvRequest.RED_OBJECT))
            if (color_flag == "blue"):
                cube_point = detect_object_service(
                    DetectObjectSrvRequest.BLUE_OBJECT).ObjList
            elif (color_flag == "red"):
                cube_point = detect_object_service(
                    DetectObjectSrvRequest.RED_OBJECT).ObjList
            elif (color_flag == "green"):
                cube_point = detect_object_service(
                    DetectObjectSrvRequest.GREEN_OBJECT).ObjList
            elif (color_flag == "all"):
                cube_point = detect_object_service(
                    DetectObjectSrvRequest.ALL_OBJECT).ObjList

            for cube in cube_point:
                if ((cube.size_width > 80 and cube.size_height > 80) or
                    (cube.size_width > 120 and cube.size_height > 40) or
                        (cube.size_width > 40 and cube.size_height > 120)):
                    cube_point = cube
                    break
            if isinstance(cube_point, list):
                continue

            rospy.loginfo("Get cube point(%dx%d:%d): %d %d" % (cube_point.size_width, cube_point.size_height,
                          cube_point.color_type, cube_point.position.x, cube_point.position.y))
        except rospy.ROSException:
            rospy.loginfo("Timeout waiting for image data.")
        except IndexError:
            rospy.loginfo("IndexError")
            continue

        # 屏幕中心偏移点
        point_offset_x = cube_point.position.x - 320
        point_offset_y = cube_point.position.y - 240

        # 判断是否居中
        if (not(abs(point_offset_x) > 40 or abs(point_offset_y) > 40)):
            if (rospy.is_shutdown()):
                break
            # cam office x:0.08 y:-0.012
            # arm.move2pose_eular(1, x + 0.08, y - 0.01, z,
            #                     pitch=math.radians(90))
            arm.move2pose_eular(1, x + 0.08, y - 0.01, 0.08,
                                pitch=math.radians(90))
            arm.gripper_catch()
            arm.move2pose_eular(1, x + 0.08, y - 0.01, z,
                                pitch=math.radians(90))
            # drop pose
            if (cube_point.color_type == DetectObjectSrvRequest.BLUE_OBJECT):
                arm.move2pose_eular(
                    1, 0.1, 0.18, 0.25, pitch=math.radians(45), yaw=math.radians(30))
            elif (cube_point.color_type == DetectObjectSrvRequest.RED_OBJECT):
                arm.move2pose_eular(
                    1, 0.2, 0.18, 0.25, pitch=math.radians(45), yaw=math.radians(30))
            elif (cube_point.color_type == DetectObjectSrvRequest.GREEN_OBJECT):
                arm.move2pose_eular(
                    1, 0.3, 0.18, 0.25, pitch=math.radians(45), yaw=math.radians(30))
            # arm.move2pose_eular(1, 0.3, 0.15, 0.2, pitch=math.radians(45))
            arm.gripper_open()
            x = default_x
            y = default_y
            z = default_z
            arm.move2pose_eular(1, x, y, z, pitch=math.radians(90))
            continue

        # 计算偏移量
        arm_office_x = (0.08 / 240) * point_offset_y + \
            ((math.sqrt(2) - 1) * 0.02) * (1 / 240 * point_offset_y + 1)
        arm_office_y = (0.105 / 320) * point_offset_x + \
            ((math.sqrt(2) - 1) * 0.02) * (1 / 320 * point_offset_x + 1)
        if (follow_flag):
            if (abs(point_offset_x) > 40):
                arm_office_y = point_offset_x / 10000.0
                # arm_office_y = arm_office_y * 0.5

            if (abs(point_offset_y) > 40):
                arm_office_x = point_offset_y / 10000.0
                # arm_office_x = arm_office_x * 0.5

            # if abs(arm_office_y) < 0.001:
            #     arm_office_y = 0.001 if arm_office_y > 0 else -0.001
            # if abs(arm_office_x) < 0.001:
            #     arm_office_x = 0.001 if arm_office_x > 0 else -0.001
        tmp_x = x - arm_office_x
        tmp_y = y - arm_office_y
        if (tmp_x > 0.27):
            tmp_x = 0.27
            rospy.logwarn("cube is out of range")
        elif (tmp_x < 0.12):
            tmp_x = 0.12
            rospy.logwarn("cube is out of range")
        if (tmp_y > 0.15):
            tmp_y = 0.15
            rospy.logwarn("cube is out of range")
        elif (tmp_y < -0.15):
            tmp_y = -0.15
            rospy.logwarn("cube is out of range")
        if (follow_flag):
            ret = arm.move2pose_eular(
                1, tmp_x, tmp_y, z, pitch=math.radians(90), wait=False)
        else:
            ret = arm.move2pose_eular(
                1, tmp_x, tmp_y, z, pitch=math.radians(90))
        if ret == True:
            x = tmp_x
            y = tmp_y
        rospy.sleep(0.1)

    arm.stop()
    # 关闭并退出moveit
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
