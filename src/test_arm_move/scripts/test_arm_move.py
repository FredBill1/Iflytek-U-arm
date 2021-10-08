#!/usr/bin/env python3

from typing import Tuple
import rospy
from arm_control.srv import *
from random import randint
from time import sleep


class ArmMove:
    def init(self) -> None:
        print("等待服务")
        rospy.wait_for_service("/arm/move_to_target_xyz")
        rospy.wait_for_service("/arm/control_manipulator")
        rospy.wait_for_service("/arm/get_pose_in_range")
        rospy.wait_for_service("/arm/get_current_xyz")

        print("连接服务")
        self.srv_move_xyz = rospy.ServiceProxy("/arm/move_to_target_xyz", Move_Target_3d)
        self.srv_control_manipulator = rospy.ServiceProxy("/arm/control_manipulator", Control_Manipulator)
        self.srv_get_pose_in_limit = rospy.ServiceProxy("/arm/get_pose_in_range", Get_Pose_In_Limit)
        self.srv_get_current_xyz = rospy.ServiceProxy("/arm/get_current_xyz", Get_Current_3d)

        print("初始化完成")

    def move_home(self, vel: float = 200.0, move_mode: str = "MOVJ") -> None:
        self.srv_move_xyz(direction="home", vel=vel, move_mode=move_mode)

    def move_xyz(self, x: float, y: float, z: float, vel: float = 200.0, move_mode: str = "MOVJ") -> None:
        self.srv_move_xyz(direction="xyz", x=x, y=y, z=z, vel=vel, move_mode=move_mode)

    def use(self, use) -> None:
        self.srv_control_manipulator(use)

    def check_xyz(self, x: float, y: float, z: float) -> bool:
        res = self.srv_get_pose_in_limit(x, y, z, True)
        return res.in_range

    def get_pos(self, is_xyz: bool = True) -> Tuple[float]:
        res = self.srv_get_current_xyz("xyz" if is_xyz else "srh")
        return res.x_s, res.y_r, res.z_h


def main():
    rospy.init_node("test_arm_move")

    arm = ArmMove()
    arm.init()

    arm.move_home()

    print("home:", arm.get_pos())

    while True:
        i = 0
        while i < 10:
            x, y, z = (randint(0, 250) for _ in range(3))
            if arm.check_xyz(x, y, z):
                print(x, y, z)
                arm.move_xyz(x, y, z)
                i += 1
        arm.move_home()
        print("sleep\n")
        sleep(3)


if __name__ == "__main__":
    main()
