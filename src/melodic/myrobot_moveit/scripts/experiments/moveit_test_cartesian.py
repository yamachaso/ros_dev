#!/usr/bin/env python2
# coding: UTF-8
import sys
from math import pi

import geometry_msgs.msg
import moveit_commander
import rospy
import tf
from geometry_msgs.msg import Quaternion, Vector3, Pose


def main():
    # MoveitCommanderの初期化
    moveit_commander.roscpp_initialize(sys.argv)

    # ノードの生成
    rospy.init_node("pose_planner")

    # eef複数含む場合には、
    move_group = moveit_commander.MoveGroupCommander("right_arm") # for open_manipulator
    # ref: https://answers.ros.org/question/334902/moveit-control-gripper-instead-of-panda_link8-eff/
    # move_group.set_end_effector_link("right_panda_hand_tip")


    place_position = (1.2, -0.63, 0.55)


    pre_pose = move_group.get_current_pose().pose
    pre_pose.position.x, pre_pose.position.y, pre_pose.position.z = place_position
    pre_pose.position.z += 0.2
    c_eef_step = 0.001
    c_jump_threshold = 0.0
    plan, plan_score = move_group.compute_cartesian_path([pre_pose], c_eef_step, c_jump_threshold)
    if plan_score < 0.5:
        print("pick failed 1...")
        return False

    print(plan)

    return True
    
    move_group.execute(plan, wait=True)

    place_pose = move_group.get_current_pose().pose
    place_pose.position.x, place_pose.position.y, place_pose.position.z = place_position
    c_eef_step = 0.001
    c_jump_threshold = 0.0
    plan, plan_score = move_group.compute_cartesian_path([place_pose], c_eef_step, c_jump_threshold)
    if plan_score < 0.5:
        print("pick failed 1...")
        return False
    
    move_group.execute(plan, wait=True)



if __name__ == "__main__":
    main()
