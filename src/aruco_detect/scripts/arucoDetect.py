#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_msgs.msg import RTVec
import cv2
import numpy as np
from math import atan2, sqrt, degrees
from typing import Tuple
from cv_bridge import CvBridge, CvBridgeError

PARAM = cv2.aruco.DICT_6X6_250
CARD_LENTH = 0.0445


def rotationVectorToEulerAngles(rvecs: np.ndarray) -> Tuple[float]:
    R = np.zeros((3, 3), dtype=np.float64)
    cv2.Rodrigues(rvecs, R)
    sy = sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    if sy > 1e-6:
        return degrees(atan2(R[2, 1], R[2, 2])), degrees(atan2(-R[2, 0], sy)), degrees(atan2(R[1, 0], R[0, 0]))
    else:
        return degrees(atan2(-R[1, 2], R[1, 1])), degrees(atan2(-R[2, 0], sy)), 0


class ArUcoDetector:
    def __init__(self) -> None:
        self.arucoDict = cv2.aruco.Dictionary_get(PARAM)

    def init(self) -> None:
        rospy.loginfo("等待 /camera_info...")
        self.cam_info: CameraInfo = rospy.wait_for_message("usb_cam/camera_info", CameraInfo)
        self.cameraMatrix = np.array(self.cam_info.P).reshape((3, 4))[:, :3]
        self.distCoeffs = np.array(self.cam_info.D)

        self.gray_sub = rospy.Subscriber("usb_cam/image_rect", Image, self.callback)
        self.cvbridge = CvBridge()

        self.twist_pub = rospy.Publisher("/aruco_vec", RTVec, queue_size=1)
        rospy.loginfo("ArUcoDetector 初始化完成\n")

    def callback(self, src: Image) -> None:
        try:
            gray = self.cvbridge.imgmsg_to_cv2(src, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        gray = cv2.equalizeHist(gray)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.arucoDict)
        if ids is not None:
            ret = cv2.aruco.estimatePoseSingleMarkers(corners, CARD_LENTH, self.cameraMatrix, self.distCoeffs)
            rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
            x, y, z = tvec
            roll, pitch, yaw = rotationVectorToEulerAngles(rvec)
            rospy.loginfo("x:%10.5f y:%10.5f z:%10.5f" % (x, y, z))
            rospy.loginfo("r:%10.5f p:%10.5f y:%10.5f\n" % (roll, pitch, yaw))
            self.twist_pub.publish(RTVec(rvec, tvec))

            cv2.aruco.drawDetectedMarkers(gray, corners)
            cv2.aruco.drawAxis(gray, self.cameraMatrix, self.distCoeffs, rvec, tvec, CARD_LENTH)  # Draw Axis
        cv2.imshow("aruco_result", gray)
        cv2.waitKey(1)


def main():
    rospy.init_node("aruco_detect", anonymous=True)
    arucoDetector = ArUcoDetector()
    arucoDetector.init()
    rospy.spin()


if __name__ == "__main__":
    main()
