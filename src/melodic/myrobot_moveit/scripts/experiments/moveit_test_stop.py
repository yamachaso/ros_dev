#!/usr/bin/env python2
# coding: UTF-8
import sys
from math import pi

import geometry_msgs.msg
import moveit_commander
import rospy
import tf
from geometry_msgs.msg import Quaternion, Vector3, Pose

import rospy
import actionlib
from moveit_msgs.msg import ExecuteTrajectoryGoal, ExecuteTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class MoveWithActionlib:
    def __init__(self):
        """ ロボットアームを動かすクラス

        Args:
            arm_name (str) : ロボットのネームスペース

        """
        self.client = actionlib.SimpleActionClient(
            '/myrobot/right_arm/right_arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def move(self, time=5):
        """ ロボットアームを動かす関数

        Args:
            angle_rad (np.array) : 関節角度(rad)
            time(int) : 動作にかける時間

        """
        joint_names = ['right_joint_1', 'right_joint_2', 'right_joint_3', 
                       'right_joint_4', 'right_joint_5', 'right_joint_6', 'right_joint_sholder']

        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = joint_names

        data = rospy.wait_for_message('/joint_states', JointState, timeout=5)

        angle_list = []
        for joint_name in joint_names:
            angle_list.append(data.position[data.name.index(joint_name)])
            
        print(angle_list)

        try:
            g.trajectory.points = [
                JointTrajectoryPoint(positions=angle_list, velocities=[0.0] * 6, time_from_start=rospy.Duration(time))]
            self.client.send_goal(g)
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        
    def stop(self):
        self.client.cancel_all_goals()


def main():
    #   初期化
    rospy.init_node('moveit_emergency_stop_example', anonymous=True)

    myrobot = MoveWithActionlib()
    myrobot.stop()



if __name__ == "__main__":
    main()
