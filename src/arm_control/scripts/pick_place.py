#!/usr/bin/env python3
# data :06.01.2020
# Author: xinnie@iflytek.com
from arm_control.srv import *
from arm_control.msg import *
from std_msgs.msg import Int32
from std_msgs.msg import Int8
import os
import sys
import rospy
import math
import numpy as np
import time

# 用于调用服务，让机械臂到达末端位置
def move_to_target_xyz_client(direction="xyz",x=None, y=None, z=None,speed=20):
    rospy.wait_for_service('/arm/move_to_target_xyz')
    try:
        move_to_target_xyz = rospy.ServiceProxy(
            '/arm/move_to_target_xyz', Move_Target_3d)  # 获得一个请求处理函数的句柄，在下面会用到
        # 直接将参数按.srv文件中的顺序填好，传入句柄，即可得到结果
        resp = move_to_target_xyz(direction, x, y, z, 0, 0, 0, False,speed,"MOVJ")
        print("resp.result",resp.result)
        return resp.result
    except:
        print("error")

# 用于设置末端执行器
def set_manipulator(end_type):
    rospy.wait_for_service('/arm/set_manipulator_type')
    try:
        set_manipulator = rospy.ServiceProxy(
            '/arm/set_manipulator_type', Set_Manipulator_Type)  # 获得一个请求处理函数的句柄，在下面会用到
        # 直接将参数按.srv文件中的顺序填好，传入句柄，即可得到结果
        resp = set_manipulator(end_type)
        return resp.result
    except:
        print("error")

# 用于调用服务，让末端执行器开合
def control_manipulator(is_open):
    rospy.wait_for_service('/arm/control_manipulator')
    try:
        control_manipulator = rospy.ServiceProxy(
            '/arm/control_manipulator', Control_Manipulator)  # 获得一个请求处理函数的句柄，在下面会用到
        # 直接将参数按.srv文件中的顺序填好，传入句柄，即可得到结果
        resp = control_manipulator(is_open)
        return resp.result
    except:
        print("error")
        
def get_current_xyz_client():
    rospy.wait_for_service('/arm/get_current_xyz')
    try:
        get_current_xyz = rospy.ServiceProxy(
            '/arm/get_current_xyz', Get_Current_3d)  # 获得一个请求处理函数的句柄，在下面会用到
        # 直接将参数按.srv文件中的顺序填好，传入句柄，即可得到结果
        resp1 = get_current_xyz("xyz")
        return resp1.x_s,resp1.y_r,resp1.z_h
    except:
        print("error")
                
if __name__ == "__main__":
    rospy.init_node('pick_and_place', anonymous=True)
    
    # 设置末端执行器
    result = set_manipulator("pump")
    print(result)

    # 到达位置1
    result = move_to_target_xyz_client("xyz",200, -30, 70)
    print(result)
    
    # z到达30mm位置
    result = move_to_target_xyz_client("z",z=30)
    print(result)
    
    # 打开吸盘
    result = control_manipulator(True)
    print(result)
    time.sleep(1)
    
    # 到达位置2
    result = move_to_target_xyz_client("xyz",240, 70, 150)
    print(result)
    
    # z到达30mm位置
    result = move_to_target_xyz_client("z",z= 30)
    print(result)
    
    # 关闭吸盘
    result = control_manipulator(False)
    print(result)
    time.sleep(1)
