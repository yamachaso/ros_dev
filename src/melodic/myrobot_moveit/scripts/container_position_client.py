#!/usr/bin/env python2
import rospy
from actionlib import SimpleActionClient
from std_msgs.msg import Bool
from detect.msg import (ContainerPositionAction, ContainerPositionGoal)


class ContainerPositionClient:
    def __init__(self, wait=True):

        self.client = SimpleActionClient('container_position_server', ContainerPositionAction)
        if wait:
            self.client.wait_for_server()


    def get(self):
        self.client.send_goal_and_wait(ContainerPositionGoal(Bool(data=True)))
        res = self.client.get_result()

        return res
    
if __name__ == "__main__":
    rospy.init_node("container_position_client", log_level=rospy.INFO)

    cli = ContainerPositionClient()
    
    print(cli.get())

    rospy.spin()