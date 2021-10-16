#!/usr/bin/env python3

import rospy
import cv2
from cv_msgs.msg import RTVec
import numpy as np
from typing import List
from ArmControl import ArmControl
from math import atan2, sin, cos, degrees


DX = 20
DY = 20
DZ = 20


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
        try:
            aruco: RTVec = rospy.wait_for_message("/aruco_vec", RTVec, 1)
        except rospy.ROSException as e:
            rospy.loginfo("ArUco notFound.")
            return False
        rospy.loginfo("xr:%10.5f yr:%10.5f zr:%10.5f" % (aruco.rvec))
        rospy.loginfo("xt:%10.5f yt:%10.5f zt:%10.5f" % tuple(v * 1000 for v in aruco.tvec))
        self.R_target2cam.append(np.array(aruco.rvec).T)
        self.t_target2cam.append(np.array(aruco.tvec).T)
        return True

    def getArm(self):
        x, y, z = self.arm.get_xyz()
        r = atan2(y, x)
        rospy.loginfo("x :%10.5f y :%10.5f z :%10.5f r :%10.5f\n" % (x, y, z, degrees(r)))
        self.R_gripper2base.append(np.array([0.0, 0.0, r]).T)
        self.t_gripper2base.append(np.array([x, y, z]).T / 1000)

    def calc(self):
        self.R_cam2gripper, self.t_cam2gripper = cv2.calibrateHandEye(
            self.R_gripper2base, self.t_gripper2base, self.R_target2cam, self.t_target2cam
        )
        print(self.R_cam2gripper)
        print(self.t_cam2gripper)
        res = np.vstack((np.hstack((self.R_cam2gripper, self.t_cam2gripper)), np.array([0, 0, 0, 1.0])))
        print(res)

    def getData(self):
        for i in range(-2, 3):
            for j in range(-2, 3):
                for k in range(-1, 4):
                    x = self.X0 + i * DX
                    y = self.Y0 + j * DY
                    z = self.Z0 + k * DZ
                    if self.arm.move_xyz(x, y, z):
                        rospy.sleep(1)
                        if not self.getAruco():
                            continue
                        self.getArm()


def main():
    rospy.init_node("EyeOnHandCalibration", anonymous=True)
    calibrator = Calibrator()
    calibrator.init()
    calibrator.getData()
    calibrator.calc()
    rospy.spin()


if __name__ == "__main__":
    main()
