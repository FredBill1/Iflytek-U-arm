#!/usr/bin/env python3

import roslib
import actionlib

roslib.load_manifest("test_cv_py")
import sys
import rospy
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from darknet_ros_msgs.msg import CheckForObjectsAction, CheckForObjectsGoal


WINNAME = "123231"


class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/darknet_ros/detection_image", Image, self.cb)
        self.received = False
        rospy.on_shutdown(self.on_shutdown)

    def cb(self, data: Image):
        self.received = True
        self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def spin(self):
        while not rospy.is_shutdown():
            if not self.received:
                rospy.sleep(2)
                continue
            cv2.imshow(WINNAME, self.img)
            cv2.waitKey(10)

    def on_shutdown(self):
        cv2.destroyAllWindows()


def main(args):
    rospy.init_node("showdetect", anonymous=True)
    ic = image_converter()
    ic.spin()
    rospy.signal_shutdown("quit")


if __name__ == "__main__":
    main(sys.argv)
