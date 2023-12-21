#!/usr/bin/env python2
import rospy
import tf
from actionlib import SimpleActionClient
from std_msgs.msg import Float64
from cart_control.msg import (CartBackandforthAction,
                              CartBackandforthGoal)


class CartBackandforthServerClient:
    def __init__(self, wait=True):

        self.client = SimpleActionClient('cart_backandforth_server', CartBackandforthAction)
        if wait:
            self.client.wait_for_server()

    def forth(self):
        self.client.send_goal_and_wait(CartBackandforthGoal(False))
        res = self.client.get_result().success

        return res
    
    def back(self):
        self.client.send_goal_and_wait(CartBackandforthGoal(True))
        res = self.client.get_result().success

        return res
    

if __name__ == "__main__":
    rospy.init_node('cart_backandforth_client')

    cli = CartBackandforthServerClient()
    

    # cli.forth()
    cli.back()

    rospy.spin()
  
