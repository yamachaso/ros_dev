#!/usr/bin/env python2
import rospy
import tf
import numpy as np
from actionlib import SimpleActionServer
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_matrix
from cart_control.msg import (CartBackandforthAction,
                              CartBackandforthGoal,
                              CartBackandforthResult)


class CartBackandforthServer:
    def __init__(self, name):
        rospy.init_node(name, log_level=rospy.INFO)

        self.server = SimpleActionServer(name, CartBackandforthAction, self.callback, False)
        self.server.start()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.speed = 1.0
        
    
    def callback(self, goal):
        try:
            back = goal.back
                
            rate = rospy.Rate(10.0)
            listener = tf.TransformListener()

            is_finished_linear = False
            is_finished_angular = False


            while not rospy.is_shutdown():
                try:
                    (trans, rot) = listener.lookupTransform('/base_link', '/ar_marker_0', rospy.Time())
                    x = trans[0]

                    # print("x : ", x)

                    cmd = Twist()

                    if not is_finished_linear:
                        if back:
                            if x < 1.12:
                                cmd.linear.x = - self.speed
                            else:
                                is_finished_linear = True
                        else: # forth
                            if x > 0.66:
                                cmd.linear.x = self.speed
                            else:
                                is_finished_linear = True
                    else:
                        mt = quaternion_matrix(rot)
                        av = np.array([-mt[0][2], mt[1][2]])
                        ev = np.array([1, 0])
                        angle = np.rad2deg(np.arccos(np.dot(av, ev) / np.linalg.norm(av)))
                        if av[1] > 0:
                            angle = - angle

                        print("angle : ", angle)

                        if angle < -1:
                            cmd.angular.z = - self.speed / 2
                        elif angle > 1:
                            cmd.angular.z = self.speed / 2
                        else:
                            is_finished_angular = True

                    print("cmd : ", cmd)                        
                    self.pub.publish(cmd)

                    if is_finished_angular:
                        break

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                
                rate.sleep()

        except Exception as err:
            rospy.logerr(err)


        self.server.set_succeeded(CartBackandforthResult(True))


if __name__ == "__main__":
    
    CartBackandforthServer("cart_backandforth_server")

    rospy.spin()