#!/usr/bin/env python2
# coding: UTF-8

import sys
import math
# import copy
import rospy
import moveit_commander
# import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("add_box_node")

scene = moveit_commander.PlanningSceneInterface(synchronous=True)

container_pose = geometry_msgs.msg.PoseStamped()
container_pose.header.frame_id = "base_link"

q = quaternion_from_euler(0.0, 0.0, -math.pi / 2)
container_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
container_pose.pose.position.x = 1
container_pose.pose.position.z = -0.4
container_name = "container"

container_filename = "/home/shin/catkin_ws/src/myrobot_description/urdf/container/close.dae"
# container_filename = "package://myrobot_description/urdf/container/open.dae"
scene.add_mesh(container_name, container_pose, container_filename, size=(0.001, 0.001, 0.001))
rospy.sleep(10)
scene.remove_world_object(container_name)
