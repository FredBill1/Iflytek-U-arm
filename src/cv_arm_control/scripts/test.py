#!/usr/bin/env python3

import rospy
import cv2
from cv_msgs.msg import RTVec
from ArmControl import ArmControl
import numpy as np
from math import sin, cos, atan2

R_cam2gripper = np.array(
    [
        [0.22173039, -0.10159043, -0.96980154],
        [0.13559011, 0.98810829, -0.07250751],
        [0.965635, -0.11541838, 0.2328683],
    ]
)
t_cam2gripper = np.array([[-0.19031578], [0.0074771], [0.0]])

C2G = np.vstack((np.hstack((R_cam2gripper, t_cam2gripper)), [0, 0, 0, 1]))
print(C2G)


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


rospy.init_node("test", anonymous=True)
arm = ArmControl()
arm.init()
obj_C = np.array([0, 0, 0, 0]).T


def callback(res: RTVec):
    global obj_C
    obj_C = np.array([list(res.tvec) + [1]]).T
    G2B = RobotMatrix(*arm.get_xyz())
    obj_G = np.dot(C2G, obj_C)
    obj_B = np.dot(G2B, obj_G)
    x, y, z, _ = np.array(obj_B.T * 1000)[0]
    print(x, y, z)


rospy.Subscriber("/aruco_vec", RTVec, callback)
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
