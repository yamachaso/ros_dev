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


    place_position = (1.35, -0.63, 0.55)

    place_pose = move_group.get_current_pose().pose
    place_pose.position.x, place_pose.position.y, place_pose.position.z = place_position
    place_pose.position.z += 0.1
    # move_group.set_position_target([1.5, -0.2, 0.2], end_effector_link="right_soft_hand_tip")
    move_group.set_pose_target(place_pose)
    # move_group.set_pose_target(place_pose, end_effector_link="right_soft_hand_tip")

    # 追記：実行可能かの確認
    plan = move_group.plan()
    if not plan.joint_trajectory.points:
        rospy.logerr("No motion plan found")

    # モーションプランの計画と実行
    move_group.go(wait=True)


    move_group.execute(plan, wait=True)

    place_pose.position.z -= 0.1
    move_group.set_pose_target(place_pose)
    plan = move_group.plan()
    if not plan.joint_trajectory.points:
        rospy.logerr("No motion plan found")
    
    move_group.execute(plan, wait=True)



if __name__ == "__main__":
    main()
