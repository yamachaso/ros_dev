#!/usr/bin/env python2
# coding: UTF-8
import sys
from math import pi

import geometry_msgs.msg
import moveit_commander
import rospy
import tf
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Float64MultiArray


def main():
    # MoveitCommanderの初期化
    moveit_commander.roscpp_initialize(sys.argv)

    # ノードの生成
    rospy.init_node("pose_planner")

    # eef複数含む場合には、
    move_group = moveit_commander.MoveGroupCommander("right_arm") # for open_manipulator
    # move_group.set_goal_joint_tolerance(0.07)

    hand_pub = rospy.Publisher('/hand_ref_pressure', Float64MultiArray, queue_size=10)


    # ref: https://answers.ros.org/question/334902/moveit-control-gripper-instead-of-panda_link8-eff/
    # move_group.set_end_effector_link("right_panda_hand_tip")
    pose = move_group.get_current_pose()
    print(pose)
    pose.pose.position.z -= 0.2
    # move_group.set_position_target([1.5, -0.2, 0.2], end_effector_link="right_soft_hand_tip")
    move_group.set_pose_target(pose, end_effector_link="right_soft_hand_tip")

    # 追記：実行可能かの確認
    plan = move_group.plan()
    if not plan.joint_trajectory.points:
        rospy.logerr("No motion plan found")

    # モーションプランの計画と実行
    move_group.go(wait=True)

    hand_msg = Float64MultiArray()
    hand_msg.data = [0.0, 0.2]
    hand_pub.publish(hand_msg)


    # 後処理
    move_group.stop()
    move_group.clear_pose_targets()

    # rospy.spin()


if __name__ == "__main__":
    main()
