#!/usr/bin/env python2
import message_filters as mf
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from actionlib import SimpleActionClient
from detect.msg import GraspDetectionAction, GraspDetectionGoal, CalcurateInsertionAction, CalcurateInsertionGoal, HandSpeedDirection
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from geometry_msgs.msg import Vector3

import numpy as np
from std_msgs.msg import Bool, Int64MultiArray, Float64, Empty, Float64MultiArray
from modules.ros.utils import call
from modules.ros.utils import multiarray2numpy
from modules.colored_print import *

from controller_manager_msgs.srv import SwitchController

from planning import MoveGroup
# rostopic pub /hand_emergency std_msgs/Int64MultiArray "data: [0, 1]"


class EmergencyClient:
    def __init__(self):
        self.pub = rospy.Publisher("/is_in_peril", Bool, queue_size=10)
        self.sub = rospy.Subscriber("/hand_emergency", Int64MultiArray, self.stop)
        self.startup_pub = rospy.Publisher('/startup/right', Empty, queue_size=1)
        self.lower_speed_pub = rospy.Publisher('/target_hand_lower_speed', HandSpeedDirection, queue_size=1)
        self.hand_pub = rospy.Publisher('/hand_ref_pressure', Float64MultiArray, queue_size=1)

        self.in_process = False


    def stop(self, msg):
        hands_value = np.array(msg.data) # int64
        if self.in_process:
            return
        if hands_value[1] == 1:
        # if hands_value[0] == 1 or hands_value[1] == 1:
            self.in_process = True
            printr("Emergency called!!")
            peril_msg = Bool()
            peril_msg.data = True
            self.pub.publish(peril_msg)

            hand_msg = Float64MultiArray()
            hand_msg.data = [0.0, 0.0]
            self.hand_pub.publish(hand_msg)

            # call("/myrobot/controller_manager/switch_controller", SwitchController,
            call("/myrobot/right_arm/controller_manager/switch_controller", SwitchController,
                start_controllers=[],
                stop_controllers=["right_cartesian_motion_controller", "right_arm_controller"],
                strictness=1, start_asap=False, timeout=0.0)
            
            rospy.sleep(1)


            empty_msg = Empty()
            self.startup_pub.publish(empty_msg)

            # call("/myrobot/controller_manager/switch_controller", SwitchController,
            call("/myrobot/right_arm/controller_manager/switch_controller", SwitchController,
                start_controllers=["right_cartesian_motion_controller"],
                stop_controllers=[""],
                strictness=1, start_asap=True, timeout=5.0)

            move_time = 1
            lower_speed = HandSpeedDirection()
            lower_speed.speed = -0.1
            lower_speed.direction = Vector3(x = 0.0, y = 0.0, z = -1.0)
            self.lower_speed_pub.publish(lower_speed)

            rospy.sleep(move_time)

            lower_speed.speed = 0
            self.lower_speed_pub.publish(lower_speed)

            rospy.sleep(move_time)

            printb("up finished")

            call("/myrobot/right_arm/controller_manager/switch_controller", SwitchController,
                start_controllers=["right_arm_controller"],
                stop_controllers=["right_cartesian_motion_controller"],
                strictness=1, start_asap=True, timeout=5.0)
            

            # rospy.sleep(1)


            mv_right_arm = MoveGroup("back_and_right_arm")
            target_name = "back_and_right_arm_start"
            target_joint_dict = mv_right_arm.get_named_target_values(target_name)
            plan = mv_right_arm.plan(target_joint_dict)
            mv_right_arm.execute(plan, wait=True)

            peril_msg.data = False
            self.pub.publish(peril_msg)

            self.in_process = False
            printr("Emergency process finished!!!")

        

if __name__ == "__main__":
    rospy.init_node("emergency_client", log_level=rospy.INFO)

    cli = EmergencyClient()
    rospy.spin()