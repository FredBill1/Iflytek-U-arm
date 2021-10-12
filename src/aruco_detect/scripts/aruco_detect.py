#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class ArUcoDetector:
    def __init__(self):
        self.graySub = rospy.Subscriber("usb_cam/image_rect", Image, self.callback)
        self.cvbridge = CvBridge()

    def callback(self, src: Image):
        try:
            cv_image = self.cvbridge.imgmsg_to_cv2(src, "passthrough")
        except CvBridgeError as e:
            print(e)
            return

        print(cv_image.shape)


def main():
    rospy.init_node("aruco_detect", anonymous=True)

    arucoDetector = ArUcoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    # cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
