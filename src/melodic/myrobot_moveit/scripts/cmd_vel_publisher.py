#!/usr/bin/env python

# rostopic pub -1 /target_hand_lower_speed std_msgs/Float64 "data: 3.14159"

import numpy as np
import quaternion
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, Float64
from detect.msg import HandSpeedDirection
import tf2_ros
import sys
import time




class cmdVelPublisher:

    def __init__(self):
        rospy.init_node('cmd_vel_publisher')
        
        self.twist = Twist()

        self.target_speed = 0.0
        self.last_target = 0.0

        self.current_speed = 0.0

        self.pub = rospy.Publisher("/cmd_vel/right", Twist, queue_size=1)
        self.sub = rospy.Subscriber("/target_hand_lower_speed", HandSpeedDirection, self.setVelocity_cb)

        rospy.Timer(rospy.Duration(0.1), self.timerCallback)


    def timerCallback(self, event):
        if self.target_speed - self.last_target > 0.0:
            # self.twist.linear.z += 0.01
            # self.twist.linear.z = min(self.twist.linear.z, self.target_speed)
            self.current_speed += 0.01
            self.current_speed = min(self.current_speed, self.target_speed)
            self.twist.linear.x = self.direction.x * self.current_speed
            self.twist.linear.y = self.direction.y * self.current_speed
            self.twist.linear.z = self.direction.z * self.current_speed


        elif self.target_speed - self.last_target < 0.0:
            # self.twist.linear.z -= 0.01
            # self.twist.linear.z = max(self.twist.linear.z, self.target_speed)
            self.current_speed -= 0.01
            self.current_speed = max(self.current_speed, self.target_speed)
            self.twist.linear.x = self.direction.x * self.current_speed
            self.twist.linear.y = self.direction.y * self.current_speed
            self.twist.linear.z = self.direction.z * self.current_speed
        self.pub.publish(self.twist)

    def setVelocity_cb(self, msg):
        self.last_target = self.target_speed
        self.target_speed = msg.speed
        self.direction = msg.direction
        


if __name__ == '__main__':
    try:
        cmd_vel_publisher = cmdVelPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
