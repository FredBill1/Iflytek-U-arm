#!/usr/bin/env python3
# data :06.01.2020
# Author: xinnie@iflytek.com
import os
import sys
import rospy
import math
import functools
import threading
import time
import rospkg
import cv2
import numpy as np
from queue import Queue, Empty
sys.path.append(os.path.join(os.path.dirname(__file__), '../'))
from uarm.wrapper import SwiftAPI
from arm_control.srv import *
from arm_control.msg import *
from std_msgs.msg import Int32
from std_msgs.msg import Int8

class ArmServer(object):
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.ID_arm = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'}, cmd_pend_size=2, callback_thread_pool_size=0)

        """
        自定义服务
        """
        srv_move_xyz = rospy.Service("move_to_target_xyz", Move_Target_3d, self.move_to_target_xyz)
        srv_get_current_xyz = rospy.Service("get_current_xyz", Get_Current_3d, self.get_current_status)
        srv_move_servo = rospy.Service("move_to_target_servo", Move_Target_Servo, self.move_to_target_servo)
        srv_get_servos_status = rospy.Service("get_servos_status", Get_Servos_Status, self.get_servos_status)
        srv_set_manipulator_type = rospy.Service("set_manipulator_type", Set_Manipulator_Type, self.set_manipulator_type)
        srv_control_manipulator = rospy.Service("control_manipulator", Control_Manipulator, self.control_manipulator)
        srv_get_mapulator_status = rospy.Service("get_manipulator_status", Get_Manipulator_Status, self.get_manipulator_status)
        srv_servo_attach = rospy.Service("set_servo_attach", Set_Servo_Attach, self.set_servo_attach)
        srv_arm_connect = rospy.Service("set_arm_connect", Set_Arm_Connect, self.set_arm_connect)
        srv_get_pose_in_limit = rospy.Service("get_pose_in_range", Get_Pose_In_Limit, self.get_pose_in_limit)
        srv_set_arm_interrupt = rospy.Service("set_arm_interrupt", Set_Arm_Interrupt, self.set_arm_interrupt)
        srv_set_led = rospy.Service("set_led", Set_Led,self.set_led)

        """
        自定义参数
        """
        self._end_type = rospy.get_param("/default_end_type") 
        self._s_none = rospy.get_param("/s_none")
        self._h_none = rospy.get_param("/h_none")
        self._s_pump = rospy.get_param("/s_pump")
        self._h_pump = rospy.get_param("/h_pump")
        self._s_holder = rospy.get_param("/s_holder")
        self._h_holder = rospy.get_param("/h_holder")
        self._s_gripper_h = rospy.get_param("/s_gripper_h")
        self._h_gripper_h = rospy.get_param("/h_gripper_h")
        self._s_gripper_v = rospy.get_param("/s_gripper_v")
        self._h_gripper_v = rospy.get_param("/h_gripper_v")

        self._move_mode = "movj"
        self._is_power_on = False
        self._is_connect = False
        self._cicle_start_point_code = [0,0,0]
        self._cicle_middle_point_code = [0,0,0]
        self._cicle_end_point_code = [0,0,0]        
        
    """
    [1] 控制机械臂到达某一固定位置,可设置的参数包括:
    direction:"xyz"、"x"、"y"、"z"、"X+"、"X-"、"Y+"、"Y-"、"Z+"、"Z-"、"srh"．
    x:直角坐标系下生效
    y:直角坐标系下生效
    z:直角坐标系下生效
    s:极坐标系下生效
    r:极坐标系下生效
    h:极坐标系下生效
    is_relative:是不是相对运动，默认是False，即非相对运动．
    move_mode:运动方式,'MOVJ'/'MOVL'/"JUMP",注意，"JUMP"下无相对运动模式
    wait:表示是否等待该动作完成,才开始下一个动作
    vel:速度,建议设置慢速，其范围为0-200，建议设置慢速如20-50
    """
    def move_to_target_xyz(self, req):  # 请求处理函数，req是请求数据包
        print("GetMoveTargetXyzCB")
        if self.ID_arm.get_arm_status()== "V0":
            return Move_Target_3dResponse(False, "E24")
        if req.vel < 0 or req.vel > 200:
            return Move_Target_3dResponse(False, "E21")
        else:
            vel = req.vel
        direction_list_p = ["X+","Y+","Z+"]
        direction_list_m = ["X-","Y-","Z-"]
        if req.direction == "home":
            result = self.ID_arm.reset(speed=vel, wait=True)
        elif req.direction == "srh":
            result = self.ID_arm.set_polar(stretch=req.x, rotation=req.y, height=req.z, speed=vel, wait=True, timeout=50)
        elif req.direction in direction_list_p:
            result = self.ID_arm.set_end_move(index =direction_list_p.index(req.direction), speed=vel,end_type = self._end_type)
        elif req.direction in direction_list_m:
            result = self.ID_arm.set_end_move(index =direction_list_m.index(req.direction), speed=-vel,end_type = self._end_type)
        elif req.direction == "xyz":
            if req.move_mode == "MOVJ":
                self._move_mode = "movj"
                self._cicle_middle_point_code = [0,0,0]
                result = self.ID_arm.set_position(
                    x=req.x, y=req.y, z=req.z, speed=vel, relative=req.is_relative, wait=True, cmd='G0', timeout=50,end_type = self._end_type)
            elif req.move_mode == "MOVL":
                self._move_mode = "movl"
                self._cicle_middle_point_code = [0,0,0]
                result = self.ID_arm.set_position(
                    x=req.x, y=req.y, z=req.z, speed=vel, relative=req.is_relative, wait=True, cmd='G1', timeout=50,end_type = self._end_type)
            elif req.move_mode == "CIRCLE_MID":
                # 有一个路径点，不做处理，仅保留
                self._cicle_middle_point_code = [round(req.x),round(req.y),round(req.z)]
                result = "OK"
            elif req.move_mode == "CIRCLE_END":
                # 起点
                result_current = self.ID_arm.get_position(wait=True)
                self._cicle_start_point_code = [round(result_current[0]),round(result_current[1]),round(result_current[2])]
                # 终点
                self._cicle_end_point_code = [round(req.x),round(req.y),round(req.z)]
                if self._cicle_middle_point_code == [0,0,0]:
                    result = "E28"
                else:
                    print(self._cicle_start_point_code,self._cicle_middle_point_code,self._cicle_end_point_code)
                    result = self.ID_arm.move_circle(self._cicle_start_point_code,self._cicle_middle_point_code,self._cicle_end_point_code,vel)
                    self._cicle_start_point_code = [0,0,0]
                    self._cicle_middle_point_code = [0,0,0]
                    self._cicle_end_point_code = [0,0,0]
            else:
                return Move_Target_3dResponse(False, "E31")
        elif req.direction == "x":
            result = self.ID_arm.set_position(
                    x=req.x, speed=vel, relative=req.is_relative, wait=True, cmd='G0', timeout=50,end_type = self._end_type)
        elif req.direction == "y":
            result = self.ID_arm.set_position(
                    y=req.y, speed=vel, relative=req.is_relative, wait=True, cmd='G0', timeout=50,end_type = self._end_type)
        elif req.direction == "z":
            result = self.ID_arm.set_position(
                    z=req.z, speed=vel, relative=req.is_relative, wait=True, cmd='G0', timeout=50,end_type = self._end_type)
        elif req.direction == "srh":
            result = self.ID_arm.set_polar(stretch=req.s, rotation=req.r, height=req.h, speed=vel, wait=True, timeout=50)
        else:
            return Move_Target_3dResponse(False, "E21")
        if result == "OK":
            return Move_Target_3dResponse(True, "")
        elif result == "TIMEOUT":
            return Move_Target_3dResponse(False, "E20")
        elif result == "uArm is not connect":
            return Move_Target_3dResponse(False, "E19")
        else:
            return Move_Target_3dResponse(False, result)

    """
    [2]获取当前末端位置,包括直角坐标系和极坐标系
    mode:可取参数为"xyz" 和"srh"
    返回:x_s  y_h  z_h表示当前的末端位置
    """
    def get_current_status(self, req):
        print("GetCurrentStatusCB")
        if self.ID_arm.get_arm_status()== "V0":
            return Get_Current_3dResponse(False, "E24", 0, 0, 0)
        if req.mode == "xyz":
            result = self.ID_arm.get_position(wait=True)
            if isinstance(result,list):
                if result != "None" and result != "TIMEOUT" and result != "uArm is not connect":
                    return Get_Current_3dResponse(True, "", result[0], result[1], result[2])
                elif result == "TIMEOUT":
                    return Get_Current_3dResponse(False, "E20", 0, 0, 0)
                elif result == "uArm is not connect":
                    return Get_Current_3dResponse(False, "E19", 0, 0, 0)
                else:
                    return Get_Current_3dResponse(False, "none_error", 0, 0, 0)                        
            elif result is None:
                return Get_Current_3dResponse(False, "E23", 0, 0, 0)
            elif result == "uArm is not connect":
                return Get_Current_3dResponse(False, "E19", 0, 0, 0)
            elif result == "TIMEOUT":
                    return Get_Current_3dResponse(False, "E20", 0, 0, 0)
            else:
                return Get_Current_3dResponse(False, str(result), 0, 0, 0)
            
        elif req.mode == "srh":
            result = self.ID_arm.get_polar(wait=True)
            if result != "None" and result != "TIMEOUT" and result != "uArm is not connect":
                return Get_Current_3dResponse(True, "", round(result[0],2), round(result[1],2), round(result[2],2))
            elif result == "TIMEOUT":
                return Get_Current_3dResponse(False, "E20", 0, 0, 0)
            else:
                return Get_Current_3dResponse(False, "none_error", 0, 0, 0)
        else:
            return Get_Current_3dResponse(False, "E21", 0, 0, 0)

    """
    [3] 控制某一关节运动到指定角度或沿某一方向按指定速度一直运动
    id:表示0-3四个电机
    angle:单位为度,取值范围为0-180度,
    vel:0-200,为末端速度，会自动根据雅克比矩阵计算单个关节速度
    """
    def move_to_target_servo(self, req):
        print("GetMoveTargetServoCB")
        if self.ID_arm.get_arm_status()== "V0":
            return Move_Target_ServoResponse(False, "E24")
        if req.vel < 0 or req.vel > 200:
            return Move_Target_ServoResponse(False, "E21")
        else:
            vel = req.vel
        servo_name = ["BUTTOM", "LEFT", "RIGHT"]
        servo_name_p = ["BUTTOM+", "LEFT+", "RIGHT-"]
        servo_name_m = ["BUTTOM-", "LEFT-", "RIGHT+"]
        all_angle = [req.angle0,req.angle1,req.angle2]
        if req.id_name in servo_name:
            result = self.ID_arm.set_servo_angle(servo_id=servo_name.index(
                req.id_name), angle=all_angle[servo_name.index(
                req.id_name)], wait=True, speed=vel, timeout=50)
        elif req.id_name == "WRIST":
            result = self.ID_arm.set_wrist(angle=req.angle3, wait=True, speed=vel)
        elif req.id_name == "ALL":
            all_angle = [req.angle0,req.angle1,req.angle2]
            result1 = self.ID_arm.set_servo_angle_together(all_angle,vel)
            result2 = self.ID_arm.set_wrist(angle=req.angle3, wait=True, speed=vel)
            if result1 == "OK" and result2 == "OK":
                result = "OK"
            elif result1 == "TIMEOUT" or result2 == "TIMEOUT":
                result = "TIMEOUT"
            elif result1 == "uArm is not connect" or result2 == "uArm is not connect":
                result = "uArm is not connect"
            else:
                result3 = result1 + result2
                if "E22" in result3:
                    result = "E22"
                elif "E21" in result3:
                    result = "E21"
                else:
                    result = result3 
        # 添加速度环控制
        elif req.id_name in servo_name_p:
            result = self.ID_arm.set_servo_move(index = servo_name_p.index(req.id_name),speed=vel,end_type = self._end_type) 
        elif req.id_name in servo_name_m:
            result = self.ID_arm.set_servo_move(index = servo_name_m.index(req.id_name),speed=-vel,end_type = self._end_type)        
        else:
            return Move_Target_ServoResponse(True, "E21")

        if result == "OK":
            return Move_Target_ServoResponse(True, "")
        elif result == "TIMEOUT":
            return Move_Target_ServoResponse(False, "E20")
        elif result == "uArm is not connect":
            return Move_Target_ServoResponse(False, "E19")
        else:
            return Move_Target_ServoResponse(False, result)

    """
    [4] 获取当前关节角度
    mode:参数为"ALL",表示同时返回四个关节角度
    返回:angles
    """
    def get_servos_status(self, req):
        print("GetServoStatusCB")
        if self.ID_arm.get_arm_status()== "V0":
            return Get_Servos_StatusResponse(False, "E24",0,0,0,0)
        result = self.ID_arm.get_servo_angle(servo_id=None, wait=True)
        if result != "None" and result != "TIMEOUT" and result != "uArm is not connect":
            result_joint4 = self.ID_arm.get_joint4_angle()
            if result_joint4[0] == "OK":
                return Get_Servos_StatusResponse(True, "", round(result[0],2), round(result[1],2), round(result[2],2),(float)(result_joint4[1][1:4]))
            elif result_joint4[0] == "TIMEOUT":
                return Get_Servos_StatusResponse(False, "E20", 0, 0, 0,0)
            else:
                return Get_Servos_StatusResponse(False, str(result_joint4[0]), 0, 0, 0,0)
        elif result == "TIMEOUT":
            return Get_Servos_StatusResponse(False, "E20", 0, 0, 0,0)
        elif result == "uArm is not connect":
            return Get_Servos_StatusResponse(False, "E19", 0, 0, 0,0)
        else:
            return Get_Servos_StatusResponse(False, "other_error", 0, 0, 0,0)

    """
    [5]设置末端执行器类型
    "none":表示无末端
    "pump":表示末端为吸盘
    "gripper_h":表示末端为夹持器且夹持器水平安装
    "gripper_v":表示末端为夹持器且夹持器竖直安装
    "holder":表示末端为笔夹
    """
    def set_manipulator_type(self,req):
        print("GetSetMapulatorTypeCB")
        ends = ["none","pump","gripper_h","gripper_v","holder"]
        offset_s = [self._s_none,self._s_pump,self._s_gripper_h,self._s_gripper_v,self._s_holder]
        offset_h = [self._h_none,self._h_pump,self._h_gripper_h,self._h_gripper_v,self._h_holder]
        if req.end_type in ends:
            self._end_type = req.end_type
            self.ID_arm.set_offset(self._end_type,offset_s[ends.index(self._end_type)],offset_h[ends.index(self._end_type)])
            return Set_Manipulator_TypeResponse(True, "")
        else:
            return Set_Manipulator_TypeResponse(False, "E30")

    """
    [6]控制末端执行器,仅仅用于吸盘与夹持器。
    used:true为使用/打开,false为关闭.
    """
    def control_manipulator(self, req):
        print("GetControlManipulatorCB")
        if self.ID_arm.get_arm_status()== "V0":
            return Control_ManipulatorResponse(False, "E24")
        result = " "
        if self._end_type == "pump":
            result = self.ID_arm.set_pump(on=req.used, wait=True)
        elif self._end_type == "gripper_h" or self._end_type == "gripper_v":
            result = self.ID_arm.set_gripper(catch=req.used, wait=True)
        else:
            return Control_ManipulatorResponse(False, "E21")
        if result == "OK":
            return Control_ManipulatorResponse(True, "")
        else:
            return Control_ManipulatorResponse(False, result)

    """
    [7]获取末端执行器状态,包括在工作中和不在工作中
    无末端和笔夹模式统一为stable
    """
    def get_manipulator_status(self, req):
        print("GetMapulatorStatusCB")
        if self.ID_arm.get_arm_status()== "V0":
            return Get_Manipulator_StatusResponse(False, "E24", "", "")
        status = ["stop", "operate_air", "operate_object"]
        if self._end_type == "pump":
            result = self.ID_arm.get_pump_status()
            if result != "None" and result != "TIMEOUT" and result != "uArm is not connect":
                return Get_Manipulator_StatusResponse(True, "", self._end_type,status[result])
            elif result == "TIMEOUT":
                return Get_Manipulator_StatusResponse(False, "time_out_error", self._end_type,"none")
            else:
                return Get_Manipulator_StatusResponse(False, "none_error", self._end_type,"none")
        elif self._end_type == "gripper_h" or self._end_type == "gripper_v":
            result = self.ID_arm.get_gripper_catch()
            if result != "None" and result != "TIMEOUT" and result != "uArm is not connect":
                return Get_Manipulator_StatusResponse(True, "", self._end_type,status[result])
            elif result == "TIMEOUT":
                return Get_Manipulator_StatusResponse(False, "time_out_error", self._end_type,"none")
            else:
                return Get_Manipulator_StatusResponse(False, "none_error", self._end_type,"none")
        elif self._end_type == "holder" or self._end_type == "none":
            return Get_Manipulator_StatusResponse(True, "", self._end_type, "stable")
        else:
            return Get_Manipulator_StatusResponse(False, "E21", "", "")
  
    """
    [8] 连接或断开机械臂
    串口通信的开启和端开
    """
    def set_arm_connect(self, req):
        print("GetSetArmConnectCB")
        if req.mode == "on":
            result = self.ID_arm.connect()
            self._is_connect = True
            return Set_Arm_ConnectResponse(True, "")
        elif req.mode == "off":
            result = self.ID_arm.disconnect()
            self._is_connect = False
            return Set_Arm_ConnectResponse(True, "")
        else:
            return Set_Arm_ConnectResponse(False, "E21")

    """
    [9] 设置电机使能/释能
    """
    def set_servo_attach(self, req):
        print("GetSetServoAttachCB")
        if self.ID_arm.get_arm_status()== "V0":
            return Set_Servo_AttachResponse(False, "E24")
        result = " "
        servo_name = ["BUTTOM", "LEFT", "RIGHT","WRIST"]
        if req.servo_name == "ALL":
            if req.mode is True:
                result = self.ID_arm.set_servo_attach(servo_id=None, wait=True)
            else:
                result = self.ID_arm.set_servo_detach(servo_id=None, wait=True)
        elif req.servo_name in servo_name:
            if req.mode is True:
                result = self.ID_arm.set_servo_attach(
                    servo_id=servo_name.index(req.servo_name), wait=True)
            else:
                result = self.ID_arm.set_servo_detach(
                    servo_id=servo_name.index(req.servo_name), wait=True)

        if result == "OK":
            return Set_Servo_AttachResponse(True, "")
        else:
            return Set_Servo_AttachResponse(False, result)

    """
    [10] 获取给定末端位置是否可以到达
    输入:
    x:x位置
    y:y位置
    z:z位置
    输出:若为true,表示可达到，否则，不可达
    """
    def get_pose_in_limit(self, req):
        print("GetPoseInRangeCB")
        pose = [req.x_s, req.y_r, req.z_h]
        result = self.ID_arm.check_pos_is_limit(
            pos=pose, is_polar=False, wait=True)
        if result != "None" and result != "TIMEOUT" and result != "uArm is not connect":
            if result == False:
              result = True
            elif result == True:
              result = False
            return Get_Pose_In_LimitResponse(True, result, "")
        elif result == "TIMEOUT":
            return Get_Pose_In_LimitResponse(False, False, "E20")
        elif result == "Arm is not connect":
            return Get_Pose_In_LimitResponse(False, False, "E19")
        else:
            return Get_Pose_In_LimitResponse(False, False, "none_error")

    """
    [11] 机械臂暂停/继续/停止运动命令
    """
    def set_arm_interrupt(self, req):
        print("GetArmInterruptCB")
        if req.type == "PAUSE":
            result = self.ID_arm.arm_pause(self._move_mode)
        elif req.type == "CONTINUE":
            result = self.ID_arm.arm_continue(self._move_mode)
        elif req.type == "STOP":
            result = self.ID_arm.arm_stop_in_move(self._move_mode)
        else:
            return Set_Arm_InterruptResponse(False, "E21")
        if result == "OK":
            return Set_Arm_InterruptResponse(True, "")
        else:
            return Set_Arm_InterruptResponse(False, result)
      

    '''
    [12].灯光控制
    灯光可表示为红绿蓝三个颜色。
    '''  
    def set_led(self,req):
        print("GetControlLedsCB")
        if req.type == "blue":
            self.ID_arm.turn_off_all()
            self.ID_arm.Blue_led(1)
        elif req.type == "green":
            self.ID_arm.turn_off_all()
            self.ID_arm.Green_led(1)
        elif req.type == "red":
            self.ID_arm.turn_off_all()
            self.ID_arm.Red_led(1)
        elif req.type == "off":
            self.ID_arm.turn_off_all()
        else:
            return Set_LedResponse(False,"E21")
        return Set_LedResponse(True,"")      
    
    """
    [13] 清除
    """
    def cleanup(self):
        print("Shutting down arm server node.")

    """
    [14] 机械臂初始化
    """
    def Arm_Init(self):
        self._is_connect = True
        ret1 = self.ID_arm.reset(speed=20, wait=True)  # 机械臂回到默认初始位置，速度设置为默认                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
        ret2 = self.ID_arm.set_mode(0)  # 设置末端执行器为无末端
        ends = ["none","pump","gripper_h","gripper_v","holder"]
        offset_s = [self._s_none,self._s_pump,self._s_gripper_h,self._s_gripper_v,self._s_holder]
        offset_h = [self._h_none,self._h_pump,self._h_gripper_h,self._h_gripper_v,self._h_holder]
        self.ID_arm.set_offset(self._end_type,offset_s[ends.index(self._end_type)],offset_h[ends.index(self._end_type)])
   
        ret3 = self.ID_arm.open_step_detect()  # 开启失步检测，若在运动过程中碰撞临时障碍物，可恢复破坏前状态并继续运行．
        self.ID_arm.RGB_Init()## 灯光初始化
        self.ID_arm.Green_led(1)



