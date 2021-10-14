#!/usr/bin/env python3

import rospy
from arm_control.srv import *
from typing import Tuple


class ArmControl:
    def init(self) -> None:
        rospy.loginfo("ArmControl - 等待服务")
        rospy.wait_for_service("/arm/move_to_target_xyz")
        rospy.wait_for_service("/arm/control_manipulator")
        rospy.wait_for_service("/arm/get_pose_in_range")
        rospy.wait_for_service("/arm/get_current_xyz")
        rospy.wait_for_service("/arm/set_servo_attach")

        rospy.loginfo("ArmControl - 连接服务")
        self.srv_move_xyz = rospy.ServiceProxy("/arm/move_to_target_xyz", Move_Target_3d)
        self.srv_control_manipulator = rospy.ServiceProxy("/arm/control_manipulator", Control_Manipulator)
        self.srv_get_pose_in_limit = rospy.ServiceProxy("/arm/get_pose_in_range", Get_Pose_In_Limit)
        self.srv_get_current_xyz = rospy.ServiceProxy("/arm/get_current_xyz", Get_Current_3d)
        self.srv_servo_attach = rospy.ServiceProxy("/arm/set_servo_attach", Set_Servo_Attach)
        rospy.loginfo("ArmControl - 初始化完成")

    def move_home(self, vel: float = 200.0, move_mode: str = "MOVJ") -> None:
        self.srv_move_xyz(direction="home", vel=vel, move_mode=move_mode)

    def move_xyz(self, x: float, y: float, z: float, vel: float = 200.0, move_mode: str = "MOVJ") -> None:
        self.srv_move_xyz(direction="xyz", x=x, y=y, z=z, vel=vel, move_mode=move_mode)

    def move_srh(self, s: float, r: float, h: float, vel: float = 200.0, move_mode: str = "MOVJ") -> None:
        self.srv_move_xyz(direction="srh", x=s, y=r, z=h, vel=vel, move_mode=move_mode)

    def use(self, use: bool) -> None:
        self.srv_control_manipulator(use)

    def attach(self, attach: bool) -> None:
        self.srv_servo_attach(attach)

    def check_xyz(self, x: float, y: float, z: float) -> bool:
        res = self.srv_get_pose_in_limit(x, y, z, True)
        return res.in_range

    def get_xyz(self) -> Tuple[float]:
        res = self.srv_get_current_xyz("xyz")
        return res.x_s, res.y_r, res.z_h

    def get_srh(self) -> Tuple[float]:
        res = self.srv_get_current_xyz("srh")
        return res.x_s, res.y_r, res.z_h


__all__ = ["ArmControl"]
