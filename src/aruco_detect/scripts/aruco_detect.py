#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class ArUcoDetector:
    def __init__(self):
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
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

        gray = cv2.flip(gray, 1)  # 机械臂上的摄像头上下是反的
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.arucoDict, parameters=self.arucoParams)
        print(corners)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(gray, corners, ids)
        cv2.imshow("aruco_result", gray)
        cv2.waitKey(1)


def main():
    rospy.init_node("aruco_detect", anonymous=True)

    arucoDetector = ArUcoDetector()
    rospy.spin()
    # cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
