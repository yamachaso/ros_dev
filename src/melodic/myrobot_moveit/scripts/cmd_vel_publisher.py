#!/usr/bin/env python

# rostopic pub -1 /target_hand_lower_speed std_msgs/Float64 "data: 3.14159"

import numpy as np
import quaternion
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, Float64

import tf2_ros
import sys
import time




class cmdVelPublisher:

    def __init__(self):
        rospy.init_node('cmd_vel_publisher')
        
        self.twist = Twist()

        self.target = 0.0
        self.last_target = 0.0

        self.pub = rospy.Publisher("/cmd_vel/right", Twist, queue_size=1)
        self.sub = rospy.Subscriber("/target_hand_lower_speed", Float64, self.setVelocity_cb)

        rospy.Timer(rospy.Duration(0.1), self.timerCallback)


    def timerCallback(self, event):
        if self.target - self.last_target > 0.0:
            self.twist.linear.z += 0.01
            self.twist.linear.z = min(self.twist.linear.z, self.target)
        elif self.target - self.last_target < 0.0:
            self.twist.linear.z -= 0.01
            self.twist.linear.z = max(self.twist.linear.z, self.target)
        self.pub.publish(self.twist)

    def setVelocity_cb(self, msg):
        self.last_target = self.target
        self.target = msg.data
        


if __name__ == '__main__':
    try:
        cmd_vel_publisher = cmdVelPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
