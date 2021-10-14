#!/usr/bin/env python3

import rospy
import cv2
from cv_msgs.msg import RTVec
import numpy as np
from typing import List
from ArmControl import ArmControl


class Calibrator:
    def __init__(self) -> None:
        self.reset()
        self.arm = ArmControl()

    def reset(self):
        self.R_gripper2base: List[np.ndarray] = []
        self.t_gripper2base: List[np.ndarray] = []
        self.R_target2cam: List[np.ndarray] = []
        self.t_target2cam: List[np.ndarray] = []

    def init(self) -> None:
        self.arm.init()
        self.arm.attach(False)

    def getData(self):
        aruco: RTVec = rospy.wait_for_message("/aruco_vec", RTVec)
        self.R_target2cam.append(aruco.rvec)
        self.t_target2cam.append(aruco.tvec)

    def calc(self):
        self.R_cam2gripper, self.t_cam2gripper = cv2.calibrateHandEye(
            self.R_gripper2base, self.t_gripper2base, self.R_target2cam, self.t_target2cam
        )
        print(self.R_cam2gripper, self.t_cam2gripper)


def main():
    rospy.init_node("EyeOnHandCalibration", anonymous=True)
    calibrator = Calibrator()
    calibrator.init()
    for i in range(50):
        calibrator.getData()
    calibrator.calc()
    rospy.spin()


if __name__ == "__main__":
    main()
