#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class ArUcoDetector:
    def __init__(self):
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self.arucoParams = cv2.aruco.DetectorParameters_create()

        self.graySub = rospy.Subscriber("usb_cam/image_rect", Image, self.callback)
        self.cvbridge = CvBridge()
        print("init ArUcoDetector done.")

    def callback(self, src: Image):
        try:
            gray = self.cvbridge.imgmsg_to_cv2(src, "passthrough")
        except CvBridgeError as e:
            print(e)
            return

        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.arucoDict, parameters=self.arucoParams)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(gray, corners)
        cv2.imshow(gray)
        cv2.waitKey(1)


def main():
    rospy.init_node("aruco_detect", anonymous=True)

    arucoDetector = ArUcoDetector()
    rospy.spin()
    # cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
