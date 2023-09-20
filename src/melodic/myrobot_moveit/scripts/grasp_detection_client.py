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
    def __init__(self, fps, image_topic, depth_topic, points_topic, ns="grasp_detection_server", ActionSpec=GraspDetectionAction, wait=True):
        # super(GraspDetectionClient, self).__init__(ns, ActionSpec)
        delay = 1 / fps * 0.5

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
            self.request = GraspDetectionGoal(img_msg, depth_msg, points_msg)
            self.request2 = CalcurateInsertionGoal(img_msg, depth_msg, points_msg)
        except Exception as err:
            rospy.logerr(err)

    def detect(self):
        last_added = self.ts.last_added
        # wait for getting new images
        while self.request is None or self.ts.last_added <= last_added:
            pass
        self.detect_client.send_goal_and_wait(self.request)
        res = self.detect_client.get_result().objects
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
    image_topic = rospy.get_param("image_topic")
    depth_topic = rospy.get_param("depth_topic")
    points_topic = rospy.get_param("points_topic")

    cli = GraspDetectionClient(
        fps=fps,
        image_topic=image_topic,
        depth_topic=depth_topic,
        points_topic=points_topic,
    )

    rate = rospy.Rate(1)

    while not rospy.is_shutdown(): 
        cli.detect()
        rate.sleep()