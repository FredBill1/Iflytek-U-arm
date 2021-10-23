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


class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("usb_cam/darknet_img", Image, queue_size=1)
        self.client = actionlib.SimpleActionClient("/darknet_ros/check_for_objects", CheckForObjectsAction)
        self.client.wait_for_server()

    def pub(self):
        data = rospy.wait_for_message("usb_cam/image_rect_color", Image)
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        img = cv2.flip(img, 1)
        # res = self.bridge.cv2_to_imgmsg(img, "rgb8")
        image = self.bridge.cv2_to_imgmsg(img, "rgb8")
        goal = CheckForObjectsGoal(image=image)
        res = self.client.send_goal_and_wait(goal)
        print(res)
        res = self.client.get_result()
        print(res)
        print(type(res))
        # self.image_pub.publish(res)
        # self.image_pub.unregister()
        # rospy.sleep(10)
        # image_pub.unregister()


def main(args):

    rospy.init_node("test_cv", anonymous=True)
    ic = image_converter()
    # input("lol")
    ic.pub()
    rospy.signal_shutdown("quit")


if __name__ == "__main__":
    main(sys.argv)
