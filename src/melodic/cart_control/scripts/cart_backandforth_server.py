#!/usr/bin/env python2
import rospy
import tf
from actionlib import SimpleActionServer
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from cart_control.msg import (CartBackandforthAction,
                              CartBackandforthGoal,
                              CartBackandforthResult)


class CartBackandforthServer:
    def __init__(self, name):
        rospy.init_node(name, log_level=rospy.INFO)

        self.server = SimpleActionServer(name, CartBackandforthAction, self.callback, False)
        self.server.start()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.speed = 1
    
    def callback(self, goal):
        try:
            back = goal.back
                
            rate = rospy.Rate(10.0)
            listener = tf.TransformListener()

            is_goal = False

            while not rospy.is_shutdown():
                try:
                    (trans, rot) = listener.lookupTransform('/base_link', '/container_base', rospy.Time())
                    x = trans[0]
                    
                    print("x : ", x)

                    cmd = Twist()

                    if back:
                        if x < 1.62:
                            cmd.linear.x = - self.speed
                        else:
                            is_goal = True
                    else: # forth
                        if x > 1.16:
                            cmd.linear.x = self.speed
                        else:
                            is_goal = True

                    self.pub.publish(cmd)

                    if is_goal:
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