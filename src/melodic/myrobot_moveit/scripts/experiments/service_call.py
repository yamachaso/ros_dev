#!/usr/bin/env python2
import rospy
from controller_manager_msgs.srv import SwitchController, LoadController

def call(ns, cls, **kwargs):
    rospy.wait_for_service(ns)
    service = rospy.ServiceProxy(ns, cls)
    response = service(**kwargs)
    print(response.ok)
    if not response.ok:
        print(response)

try:
    # When calling the switch_controller service here, I need to ensure that the controller is loaded.
    # When calling the service via cmdline (rosservice call /controller_manager/switch_controller ...) it works without loading!?
    # call("/controller_manager/load_controller", LoadController,
    #      name="position_joint_trajectory_controller")

    call("/myrobot/right_arm/controller_manager/switch_controller", SwitchController,
         start_controllers=["right_arm_controller"],
         stop_controllers=["right_cartesian_motion_controller"],
         strictness=1, start_asap=False, timeout=0.0)

except rospy.ServiceException as e:
    print("Service call failed: %s" % e)

    