#!/usr/bin/env python3

import rospy
import cv2
from cv_msgs.msg import RTVec
from ArmControl import ArmControl
import numpy as np
from math import sin, cos, atan2

C2G = np.array(
    [
        [0.0, -1.0, 0.0, 0.065],
        [-1.12, 0.0, 0.0, 0.018],
        [0.0, 0.0, -1.0, 0.05],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


def RobotMatrix(px, py, pz):
    r = atan2(py, px)
    mr = np.matrix([[cos(r), -sin(r), 0, 0], [sin(r), cos(r), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    mt = np.hstack((np.eye(4, 3), np.array([[px], [py], [pz], [1000]]) / 1000))
    return np.dot(mt, mr)


# def RobotMatrix(px, py, pz):  # arm end
#     angle_z = atan2(py, px)  # error px
#     robotMatrix = np.matrix(
#         [
#             [cos(angle_z), -sin(angle_z), 0],
#             [sin(angle_z), cos(angle_z), 0],
#             [0, 0, 1],
#         ]
#     )
#     rm1 = np.append(robotMatrix, np.array([[px], [py], [pz]]) / 1000, axis=1)
#     rm2 = np.append(rm1, [[0, 0, 0, 1]], axis=0)
#     return rm2
#     return np.linalg.inv(rm2)


rospy.init_node("test", anonymous=True)
arm = ArmControl()
arm.init()
obj_C = np.array([[0, 0, 0, 1]]).T


def callback(res: RTVec):
    global obj_C
    obj_C = np.array([list(res.tvec) + [1]]).T


rospy.Subscriber("/aruco_vec", RTVec, callback)


# arm.attach(False)
# while True:
#     G2B = RobotMatrix(*arm.get_xyz())
#     obj_G = np.dot(C2G, obj_C)
#     obj_B = np.dot(G2B, obj_G)
#     print(obj_C)
#     print(obj_G)
#     print(obj_B)
#     x, y, z, _ = np.array(obj_B.T * 1000)[0]
#     print(x, y, z)
#     print()
#     rospy.sleep(1)

while True:
    input()
    G2B = RobotMatrix(*arm.get_xyz())
    obj_G = np.dot(C2G, obj_C)
    obj_B = np.dot(G2B, obj_G)
    print(obj_C)
    print(obj_G)
    print(obj_B)
    x, y, z, _ = np.array(obj_B.T * 1000)[0]
    arm.move_xyz(x, y, 20)
    input()
    arm.move_home()

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
