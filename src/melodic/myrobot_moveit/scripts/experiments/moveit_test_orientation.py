#!/usr/bin/env python2
# coding: UTF-8
import sys
from math import pi

import geometry_msgs.msg
import moveit_commander
import rospy
import tf
from geometry_msgs.msg import Quaternion, Vector3
from tf.transformations import quaternion_from_euler
import math
import numpy as np
# from modules.colored_print import * 

def main():
    # MoveitCommanderの初期化
    moveit_commander.roscpp_initialize(sys.argv)

    # ノードの生成
    rospy.init_node("pose_planner")

    # eef複数含む場合には、
    move_group = moveit_commander.MoveGroupCommander("right_arm") # for open_manipulator
    # ref: https://answers.ros.org/question/334902/moveit-control-gripper-instead-of-panda_link8-eff/
    # move_group.set_end_effector_link("right_panda_hand_tip")
    move_group.set_planner_id('RRTConnectkConfigDefault')
    pose = move_group.get_current_pose()
    print(pose)

    angle = 10
    rpy = (np.radians(-30), np.radians(0), math.pi + np.radians(-angle)) # contact : 1
    rpy = (np.radians(30), np.radians(0), math.pi + np.radians(angle)) # contact : 4
    rpy = (np.radians(0), np.radians(angle), math.pi) # contact : 2
    rpy = (np.radians(60), np.radians(-angle), math.pi) # contact : 8
    rpy = (np.radians(-15), np.radians(angle), math.pi + np.radians(-angle)) # contact : 3
    rpy = (np.radians(15), np.radians(angle), math.pi + np.radians(angle)) # contact : 6
    rpy = (np.radians(45), np.radians(-angle), math.pi + np.radians(angle)) # contact : 12
    rpy = (np.radians(-45), np.radians(-angle), math.pi + np.radians(-angle)) # contact : 9


    
    # rpy = (math.pi, np.radians(0), np.radians(angle))
    # rpy = (180, 0, 0)
    q = quaternion_from_euler(rpy[0], rpy[1], rpy[2], axes="szyx") # x ,y, zの順で回転。引数は 3 ,2 ,1の順番
    # q = euler_to_quaternion(rpy[0], rpy[1], rpy[2])
    orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    print("orientation : ", orientation)


    pose.pose.orientation = orientation
    # move_group.set_position_target([1.5, -0.2, 0.2], end_effector_link="right_soft_hand_tip")
    move_group.set_pose_target(pose, end_effector_link="right_soft_hand_tip")


    print(move_group.get_pose_reference_frame())
    # 追記_from_euler
    # plan = move_group.plan()
    # if not plan.joint_trajectory.points:
    #     rospy.logerr("No motion plan found")

    # モーションプランの計画と実行
    move_group.go(wait=True)

    # 後処理
    move_group.stop()
    move_group.clear_pose_targets()

    # rospy.spin()


if __name__ == "__main__":
    main()
