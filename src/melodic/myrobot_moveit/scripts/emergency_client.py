#!/usr/bin/env python2
import message_filters as mf
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from actionlib import SimpleActionClient
from detect.msg import GraspDetectionAction, GraspDetectionGoal, CalcurateInsertionAction, CalcurateInsertionGoal
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction

import numpy as np
from std_msgs.msg import Bool, Int64MultiArray, Float64, Empty
from modules.ros.utils import call
from modules.ros.utils import multiarray2numpy

from controller_manager_msgs.srv import SwitchController

from chikadsuite_grasp import MoveGroup
# rostopic pub /hand_emergency std_msgs/Int64MultiArray "data: [0, 2]"


class EmergencyClient:
    def __init__(self):
        self.pub = rospy.Publisher("/is_in_peril", Bool, queue_size=10)
        self.sub = rospy.Subscriber("/hand_emergency", Int64MultiArray, self.stop)


    def stop(self, msg):
        hands_value = np.array(msg.data) # int64
        print("called")
        if hands_value[0] == 1 or hands_value[1] == 1:

            peril_msg = Bool()
            peril_msg.data = True
            self.pub.publish(peril_msg)


            print("a")
            # call("/myrobot/controller_manager/switch_controller", SwitchController,
            call("/myrobot/right_arm/controller_manager/switch_controller", SwitchController,
                start_controllers=[],
                stop_controllers=["right_cartesian_motion_controller", "right_arm_controller"],
                strictness=1, start_asap=False, timeout=0.0)
            
            print("b")
            # rospy.sleep(5)
            print("c")

            # startup_pub = rospy.Publisher('/startup/right', Empty, queue_size=1)
            # empty_msg = Empty()
            # startup_pub.publish(empty_msg)

            # # call("/myrobot/controller_manager/switch_controller", SwitchController,
            # call("/myrobot/right_arm/controller_manager/switch_controller", SwitchController,
            #     start_controllers=["right_cartesian_motion_controller"],
            #     stop_controllers=[],
            #     strictness=1, start_asap=False, timeout=0.0)
            
            # print("d")

            # print("emergency up execution")
            # move_time = 1
            # lower_speed_pub = rospy.Publisher('/target_hand_lower_speed', Float64, queue_size=1)
            # lower_speed = Float64()
            # lower_speed.data = 0.1
            # lower_speed_pub.publish(lower_speed)

            # print("e")
            # rospy.sleep(move_time)

            # lower_speed.data = 0
            # lower_speed_pub.publish(lower_speed)

            # print("f")
            # rospy.sleep(move_time + 0.05)



            # call("/myrobot/controller_manager/switch_controller", SwitchController,
            call("/myrobot/right_arm/controller_manager/switch_controller", SwitchController,
                start_controllers=["right_arm_controller"],
                stop_controllers=[""],
                strictness=1, start_asap=False, timeout=0.0)
            
            print("g")

            mv_right_arm = MoveGroup("right_arm")
            target_name = "right_arm_start"
            target_joint_dict = mv_right_arm.get_named_target_values(target_name)
            plan = mv_right_arm.plan(target_joint_dict)
            print("h")
            mv_right_arm.execute(plan, wait=True)

            print("i")
            peril_msg.data = False
            self.pub.publish(peril_msg)

        

if __name__ == "__main__":
    rospy.init_node("emergency_client", log_level=rospy.INFO)

    cli = EmergencyClient()
    rospy.spin()