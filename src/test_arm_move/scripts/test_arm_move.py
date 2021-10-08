#!/usr/bin/env python3

import rospy
from arm_control.srv import *

["direction", "x", "y", "z", "s", "r", "h", "is_relative", "vel", "move_mode"]


def main():
    rospy.init_node("test_arm_move")

    print("等待服务")
    rospy.wait_for_service("/arm/move_to_target_xyz")
    rospy.wait_for_service("/arm/control_manipulator")

    print("连接服务")
    srv_move_xyz = rospy.ServiceProxy("/arm/move_to_target_xyz", Move_Target_3d)
    srv_control_manipulator = rospy.ServiceProxy("/arm/control_manipulator", Control_Manipulator)

    def move_xyz(x: float, y: float, z: float):
        srv_move_xyz(direction="xyz", x=x, y=y, z=z, vel=200, move_mode="MOVJ")

    move_xyz(200, 0, 0)


if __name__ == "__main__":
    main()