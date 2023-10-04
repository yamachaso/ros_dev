#!/usr/bin/env python2
import message_filters as mf
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from actionlib import SimpleActionClient
from detect.msg import GraspDetectionAction, GraspDetectionGoal, CalcurateInsertionAction, CalcurateInsertionGoal
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction

from std_msgs.msg import Empty
from modules.ros.utils import call

from controller_manager_msgs.srv import SwitchController


class EmergencyClient:
    def __init__(self):
        # self.left_client = SimpleActionClient(
        #     '/myrobot/left_arm/left_arm_controller/follow_joint_trajectory',
        #     FollowJointTrajectoryAction)

        # self.right_client = SimpleActionClient(
        #     '/myrobot/right_arm/right_arm_controller/follow_joint_trajectory',
        #     FollowJointTrajectoryAction)

        # self.left_client.wait_for_server()
        # self.right_client.wait_for_server()

        self.sub = rospy.Subscriber("/target_hand_lower", Empty, self.stop)


    def stop(self):
        call("/myrobot/right_arm/controller_manager/switch_controller", SwitchController,
            start_controllers=[],
            stop_controllers=["right_cartesian_motion_controller", "right_arm_controller"],
            strictness=1, start_asap=False, timeout=0.0)
        

if __name__ == "__main__":
    rospy.init_node("emergency_client", log_level=rospy.INFO)

    cli = EmergencyClient()
    cli.stop()

    # rate = rospy.Rate(1)

    # while not rospy.is_shutdown(): 
    #     rate.sleep()