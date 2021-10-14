#!/usr/bin/env python3

import rospy
import cv2
from cv_msgs.msg import RTVec
from ArmControl import ArmControl
import numpy as np

R_cam2gripper = np.array(
    [
        [0.02754638, 0.10760802, -0.99381171],
        [0.15763517, 0.98128193, 0.11062064],
        [0.98711314, -0.15970688, 0.01006795],
    ]
)
t_cam2gripper = np.array([[-6.36468923e01], [-4.10994712e-02], [0.00000000e00]])


def main():
    rospy.init_node("test", anonymous=True)

    arm = ArmControl()
    arm.init()

    while True:
        input()
        res = rospy.wait_for_message("/aruco_vec", RTVec)
        rvec, tvec = np.array(res.rvec).T, np.array(res.tvec).T
        tvec += t_cam2gripper
        tvec *= 1000
        x, y, z = np.array(arm.get_xyz()) + tvec.T
        arm.move_xyz(x, y, z)
        input()
        arm.move_home()

    rospy.spin()


if __name__ == "__main__":

    main()
