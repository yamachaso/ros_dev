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
            depth = self.bridge.imgmsg_to_cv2(depth_msg)
            print("depth mean : ", depth.mean())
        except Exception as err:
            rospy.logerr(err)




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

    rospy.spin()