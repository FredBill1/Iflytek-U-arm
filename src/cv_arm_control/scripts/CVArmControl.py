#!/usr/bin/env python3
import rospy
from ArmControl import ArmControl
from sensor_msgs.msg import CameraInfo
import numpy as np
from math import atan2, sin, cos
from typing import Optional
from C2G import C2G

# C2G = np.array(
#     [
#         [0.0, -1.12, 0.0, 0.065],
#         [-1.17, 0.0, 0.0, 0.018],
#         [0.0, 0.0, -1.0, 0.05],
#         [0.0, 0.0, 0.0, 1.0],
#     ]
# )


def G2B(x: float, y: float, z: float):
    r = atan2(y, x)
    return np.array(
        [
            [cos(r), -sin(r), 0.0, x],
            [sin(r), cos(r), 0.0, y],
            [0.0, 0.0, 1.0, z],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


class CVArmControl(ArmControl):
    def init(self):
        rospy.loginfo("Wait for /camera_info...")
        self.cam_info: CameraInfo = rospy.wait_for_message("usb_cam/camera_info", CameraInfo)
        C2U = np.array(self.cam_info.K).reshape((3, 3))
        self.U2C = np.linalg.inv(C2U)
        self.distCoeffs = np.array(self.cam_info.D)

        super().init()

        rospy.loginfo("ArmController init done")

    def move3d(self, x: float, y: float, z: float, z_set: Optional[int] = None):
        c = np.array([[x], [y], [z], [1.0]])
        g = np.dot(C2G, c)
        b = np.dot(G2B(*(v / 1000 for v in self.get_xyz())), g)
        x, y, z, _ = np.array(b.T * 1000)[0]
        rospy.loginfo("move3d x:%10.2f y:%10.2f z:%10.2f" % (x, y, z))
        self.move_xyz(x, y, z if z_set is None else z_set)
        rospy.loginfo("move3d done.")

    def move2d(self, x: float, y: float, z: float, z_set: Optional[int] = None):
        u = np.array([[x], [y], [1.0]]) * z
        x, y, z = np.dot(self.U2C, u).T[0]
        rospy.loginfo("move2d x:%10.5f y:%10.5f z:%10.5f" % (x, y, z))
        self.move3d(x, y, z, z_set)


def main():
    rospy.init_node("cv_arm_control")


if __name__ == "__main__":
    main()
