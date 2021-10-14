#!/usr/bin/env python3

import rospy
import cv2
from cv_msgs.msg import RTVec
from ArmControl import ArmControl
import numpy as np
from math import sin, cos, atan2

R_cam2gripper = np.array(
    [
        [0.88001818, -0.08434561, -0.46739044],
        [-0.08377916, 0.94110364, -0.32757441],
        [0.46749231, 0.32742901, 0.82112191],
    ]
)
t_cam2gripper = np.array([[-0.16173787], [0.21854771], [0.0]])
C2G = np.vstack((np.hstack((R_cam2gripper, t_cam2gripper)), [0, 0, 0, 1]))


def RobotMatrix(px, py, pz):  # arm end
    angle_z = atan2(py, px)  # error px
    robotMatrix = np.matrix(
        [
            [cos(angle_z), -sin(angle_z), 0],
            [sin(angle_z), cos(angle_z), 0],
            [0, 0, 1],
        ]
    )
    rm1 = np.append(robotMatrix, np.array([[px], [py], [pz]]) / 1000, axis=1)
    rm2 = np.append(rm1, [[0, 0, 0, 1]], axis=0)
    return rm2


def main():
    rospy.init_node("test", anonymous=True)

    arm = ArmControl()
    arm.init()

    while True:
        res: RTVec = rospy.wait_for_message("/aruco_vec", RTVec)
        G2B = RobotMatrix(*arm.get_xyz())
        obj_C = np.array([list(res.tvec) + [1]]).T
        obj_G = np.dot(C2G, obj_C)
        obj_B = np.dot(G2B, obj_G)
        print(obj_C)
        print(obj_G)
        print(obj_B)
        x, y, z, _ = map(int, np.array(obj_B.T * 1000)[0])
        print("x:%5d y:%5d z:%5d" % (x, y, z))
        arm.move_xyz(x, y, z)
        arm.move_home()

    rospy.spin()


if __name__ == "__main__":

    main()
