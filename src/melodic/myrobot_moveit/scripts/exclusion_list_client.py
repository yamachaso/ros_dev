#!/usr/bin/env python2
import numpy as np

from actionlib import SimpleActionClient
from detect.msg import ExclusionListAction, ExclusionListGoal
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Int32MultiArray
from modules.ros.utils import multiarray2numpy


class ExclusionListClient(SimpleActionClient, object):
    def __init__(self, ns="exclusion_list_server", ActionSpec=ExclusionListAction):
        super(ExclusionListClient, self).__init__(ns, ActionSpec)
        self.wait_for_server()

    def add(self, arm_index, u, v):
        self.send_goal_and_wait(ExclusionListGoal(arm_index, u, v, False, False))
    
    def ref(self, arm_index):
        self.send_goal_and_wait(ExclusionListGoal(arm_index, 0, 0, True, False))
        res = multiarray2numpy(int, np.int32, self.get_result().exclusion_points)
        return res
    
    def clear(self, arm_index):
        self.send_goal_and_wait(ExclusionListGoal(arm_index, 0, 0, False, True))