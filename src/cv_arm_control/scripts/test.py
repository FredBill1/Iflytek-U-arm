#!/usr/bin/env python3

import rospy
import cv2
from cv_msgs.msg import RTVec
from ArmControl import ArmControl
import numpy as np
from math import sin, cos, atan2

C2G = np.array(
    [
        [0.99811838, -0.00673262, -0.06094566, -0.20550576],
        [0.00909799, 0.99921266, 0.0386173, 0.19904189],
        [0.06063768, -0.03909912, 0.99739377, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


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
    # return rm2
    return np.linalg.pinv(rm2)


rospy.init_node("test", anonymous=True)
arm = ArmControl()
arm.init()
obj_C = np.array([[0, 0, 0, 1]]).T


def callback(res: RTVec):
    global obj_C
    obj_C = np.array([list(res.tvec) + [1]]).T


rospy.Subscriber("/aruco_vec", RTVec, callback)


arm.attach(False)
while True:
    G2B = RobotMatrix(*arm.get_xyz())
    obj_G = np.dot(C2G, obj_C)
    obj_B = np.dot(G2B, obj_G)
    print(obj_C)
    print(obj_G)
    print(obj_B)
    x, y, z, _ = np.array(obj_B.T * 1000)[0]
    print(x, y, z)
    print()
    rospy.sleep(1)

# while True:
#     input()
#     G2B = RobotMatrix(*arm.get_xyz())
#     obj_G = np.dot(C2G, obj_C)
#     obj_B = np.dot(G2B, obj_G)
#     print(obj_C)
#     print(obj_G)
#     print(obj_B)
#     x, y, z, _ = np.array(obj_B.T * 1000)[0]
#     arm.move_xyz(x, y, 30)
#     input()
#     arm.move_home()

rospy.spin()
# G2B = RobotMatrix(*arm.get_xyz())
# obj_G = np.dot(C2G, obj_C)
# obj_B = np.dot(G2B, obj_G)
# print(obj_C)
# print(obj_G)
# print(obj_B)
# x, y, z, _ = np.array(obj_B.T * 1000)[0]
# print("x:%5.1f y:%5.1f z:%5.1f" % (x, y, z))
# arm.move_xyz(x, y, z)
# arm.move_home()
# rospy.spin()
