#!/usr/bin/env python3

import rospy
from arm_control.srv import *
from typing import Tuple

ERR_MEG = {
    "E19": "机械臂未连接",
    "E20": "超时",
    "E21": "参数错误",
    "E22": "位置超限制",
    "E23": "获取末端状态时超时",
    "E24": "电源未连接",
    "E27": "释能下运动",
    "E28": "圆弧模式下未设置中间点",
    "E30": "末端执行器类型有误",
    "E31": "控制机械臂运动时侯给出的运动模式有误",
}


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

        self.attach(True)
        self.move_home()

        rospy.loginfo("ArmControl - 归位")

    def log_move_ret(self, ret: Move_Target_3dResponse) -> None:
        if ret.success:
            rospy.loginfo("ArmControl - move succeeded")
        else:
            rospy.logerr(f"ArmControl - {ret.message} : {ERR_MEG[ret.message]}")

    def move_xyz(self, x: float, y: float, z: float, vel: float = 150.0, move_mode: str = "MOVJ") -> bool:
        ret = self.srv_move_xyz(direction="xyz", x=x, y=y, z=z, vel=vel, move_mode=move_mode)
        self.log_move_ret(ret)
        return ret.success

    def move_srh(self, s: float, r: float, h: float, vel: float = 150.0, move_mode: str = "MOVJ") -> bool:
        ret = self.srv_move_xyz(direction="srh", x=s, y=r, z=h, vel=vel, move_mode=move_mode)
        self.log_move_ret(ret)
        return ret.success

    def move_home(self, vel: float = 150.0, move_mode: str = "MOVJ") -> None:
        # self.srv_move_xyz(direction="home", vel=vel, move_mode=move_mode)
        self.move_xyz(179, 0, 118, vel, move_mode)

    def move_home2(self, vel: float = 150.0, move_mode: str = "MOVJ") -> None:
        self.move_xyz(0.001, 280.0, 20, vel, move_mode)

    def move_xyz_relative(self, x: float, y: float, z: float, vel: float = 150.0, move_mode: str = "MOVJ") -> bool:
        ret = self.srv_move_xyz(direction="xyz", x=x, y=y, z=z, vel=vel, move_mode=move_mode, is_relative=True)
        self.log_move_ret(ret)
        return ret.success

    def use(self, use: bool) -> None:
        self.srv_control_manipulator(use)

    def attach(self, attach: bool) -> None:
        self.srv_servo_attach("ALL", attach)

    def check_xyz(self, x: float, y: float, z: float) -> bool:
        res = self.srv_get_pose_in_limit(x, y, z, True)
        return res.in_range

    def check_srh(self, s: float, r: float, h: float) -> bool:
        res = self.srv_get_pose_in_limit(s, r, h, False)
        return res.in_range

    def get_xyz(self) -> Tuple[float]:
        res = self.srv_get_current_xyz("xyz")
        return res.x_s, res.y_r, res.z_h

    def get_srh(self) -> Tuple[float]:
        res = self.srv_get_current_xyz("srh")
        return res.x_s, res.y_r, res.z_h


__all__ = ["ArmControl"]
