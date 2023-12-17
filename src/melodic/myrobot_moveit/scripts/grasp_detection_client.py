#!/usr/bin/env python2
import message_filters as mf
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from actionlib import SimpleActionClient
from detect.msg import GraspDetectionAction, GraspDetectionGoal, CalcurateInsertionAction, CalcurateInsertionGoal

# ref: https://qiita.com/ynott/items/8acd03434569e23612f1
# class GraspDetectionClient(SimpleActionClient, object):
class GraspDetectionClient:
    def __init__(self, arm_index, fps, image_topic, depth_topic, points_topic, ns="grasp_detection_server", ActionSpec=GraspDetectionAction, wait=True):
        # super(GraspDetectionClient, self).__init__(ns, ActionSpec)
        delay = 1 / fps * 0.5

        self.arm_index = arm_index

        # Topics
        self.points_topic = points_topic
        # Subscribers
        subscribers = [mf.Subscriber(topic, Image) for topic in (image_topic, depth_topic)]
        subscribers.append(mf.Subscriber(points_topic, PointCloud2))
        # Others
        self.bridge = CvBridge()
        self.request = None

        self.ts = mf.ApproximateTimeSynchronizer(subscribers, 10, delay)
        # self.ts = mf.ApproximateTimeSynchronizer(subscribers, 1, delay)
        self.ts.registerCallback(self.callback)

        self.detect_client = SimpleActionClient('detect_server', GraspDetectionAction)
        self.calcurate_insertion_client = SimpleActionClient('calcurate_insertion_server', CalcurateInsertionAction)

        if wait:
            self.detect_client.wait_for_server()
            self.calcurate_insertion_client.wait_for_server()

    # def callback(self, img_msg, depth_msg):
    def callback(self, img_msg, depth_msg, points_msg):
        try:
            # points_msg = rospy.wait_for_message(self.points_topic, PointCloud2)
            self.request = GraspDetectionGoal(self.arm_index, img_msg, depth_msg, points_msg)
            self.request2 = CalcurateInsertionGoal(self.arm_index, img_msg, depth_msg, points_msg)
        except Exception as err:
            rospy.logerr(err)

    def detect(self):
        last_added = self.ts.last_added
        # wait for getting new images
        while self.request is None or self.ts.last_added <= last_added:
            pass
        self.detect_client.send_goal_and_wait(self.request)
        res = self.detect_client.get_result().object
        return res
    
    def calcurate_insertion(self):
        last_added = self.ts.last_added
        # wait for getting new images
        while self.request2 is None or self.ts.last_added <= last_added:
            pass
        self.calcurate_insertion_client.send_goal_and_wait(self.request2)
        res = self.calcurate_insertion_client.get_result()

        return res


if __name__ == "__main__":
    rospy.init_node("grasp_detection_client", log_level=rospy.INFO)

    fps = rospy.get_param("fps")
    left_image_topic = rospy.get_param("left_image_topic")
    left_depth_topic = rospy.get_param("left_depth_topic")
    left_points_topic = rospy.get_param("left_points_topic")
    right_image_topic = rospy.get_param("right_image_topic")
    right_depth_topic = rospy.get_param("right_depth_topic")
    right_points_topic = rospy.get_param("right_points_topic")

    cli = GraspDetectionClient(
        arm_index=1,
        fps=fps,
        image_topic=left_image_topic,
        depth_topic=left_depth_topic,
        points_topic=left_points_topic,
    )

    rate = rospy.Rate(5)

    while not rospy.is_shutdown(): 
        cli.detect()
        rate.sleep()