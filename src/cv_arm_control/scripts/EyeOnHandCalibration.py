#!/usr/bin/env python3

import rospy
import cv2
from cv_msgs.msg import RTVec
import numpy as np
from typing import List
from ArmControl import ArmControl
from math import atan2
from time import sleep


DX = 15
DY = 20
DZ = 15


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
        self.X0, self.Y0, self.Z0 = self.arm.get_xyz()

    def getAruco(self):
        aruco: RTVec = rospy.wait_for_message("/aruco_vec", RTVec, 1)
        self.R_target2cam.append(np.array(aruco.rvec).T)
        self.t_target2cam.append(np.array(aruco.tvec).T)

    def getArm(self):
        x, y, z = self.arm.get_xyz()
        r = atan2(y, x)
        rospy.loginfo("x:%10.5f y:%10.5f z:%10.5f r:%10.5f" % (x, y, z, r))
        self.R_gripper2base.append(np.array([0.0, 0.0, r]).T)
        self.t_gripper2base.append(np.array([x, y, z]).T / 3)

    def calc(self):
        self.R_cam2gripper, self.t_cam2gripper = cv2.calibrateHandEye(
            self.R_gripper2base, self.t_gripper2base, self.R_target2cam, self.t_target2cam
        )
        print(self.R_cam2gripper, self.t_cam2gripper)

    def calibrate(self):
        for i in range(-2, 3):
            for j in range(-2, 3):
                for k in range(-2, 3):
                    x = self.X0 + i * DX
                    y = self.Y0 + j * DY
                    z = self.Z0 + k * DZ
                    if self.arm.check_xyz(x, y, z):
                        self.arm.move_xyz(x, y, z)
                        rospy.sleep(0.1)
                        self.getAruco()
                        self.getArm()
        self.calc()


def main():
    rospy.init_node("EyeOnHandCalibration", anonymous=True)
    calibrator = Calibrator()
    calibrator.init()
    calibrator.calibrate()
    rospy.spin()


if __name__ == "__main__":
    main()
