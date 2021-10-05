#!/usr/bin/env python3
# coding=utf-8
# data :06.01.2020
# Author: xinnie@iflytek.com
from geometry_msgs.msg import *
from arm_control.srv import *
from std_msgs.msg import Int32
from std_msgs.msg import Int8
import os
import sys
import rospy
import math
import numpy as np
import time

global center

location_x = 0
location_y = 0
location_z = 0

cameralist = []
R1Tlist = []


def RobotMatrix(px, py, pz):  # arm end
    pi = math.pi
    angle_z = math.atan2(py, px)  # error px

    robotMatrix = np.matrix([
        [math.cos(angle_z), -math.sin(angle_z), 0],
        [math.sin(angle_z), math.cos(angle_z), 0],
        [0, 0, 1]
    ])

    rm1 = np.append(robotMatrix, [[px], [py], [pz]], axis=1)
    rm2 = np.append(rm1, [[0, 0, 0, 1]], axis=0)
    return rm2


'''
作用:订阅二维码检测节点发布的话题
'''


def Qcode_in_cam_callback(msg):
    global location_x, location_y, location_z
    location_x = msg.linear.x
    location_y = msg.linear.y
    location_z = msg.linear.z


def move_to_target_xyz_client(x, y, z):
    rospy.wait_for_service('/arm/move_to_target_xyz')
    try:
        move_to_target_xyz = rospy.ServiceProxy(
            '/arm/move_to_target_xyz', Move_Target_3d)  # 获得一个请求处理函数的句柄，在下面会用到
        # 直接将参数按.srv文件中的顺序填好，传入句柄，即可得到结果
        resp1 = move_to_target_xyz("xyz", x, y, z, 0, 0, 0, 0, 20, "MOVJ")
        return resp1.result
    except:
        print("error")


def get_current_xyz_client():
    rospy.wait_for_service('/arm/get_current_xyz')
    try:
        get_current_xyz = rospy.ServiceProxy(
            '/arm/get_current_xyz', Get_Current_3d)  # 获得一个请求处理函数的句柄，在下面会用到
        # 直接将参数按.srv文件中的顺序填好，传入句柄，即可得到结果
        resp1 = get_current_xyz("xyz")
        return resp1.x_s, resp1.y_r, resp1.z_h
    except:
        print("error")


def get_current_angle():
    rospy.wait_for_service('/arm/get_servos_status')
    try:
        get_current_angles = rospy.ServiceProxy(
            '/arm/get_servos_status', Get_Servos_Status)  # 获得一个请求处理函数的句柄，在下面会用到
        # 直接将参数按.srv文件中的顺序填好，传入句柄，即可得到结果
        resp2 = get_current_angles()
        return resp2.joint1
    except:
        print("error")


if __name__ == "__main__":
    rospy.init_node('eye_hand_calibration_result_vertily', anonymous=True)
    rospy.Subscriber("qcode_detect_result", Twist, Qcode_in_cam_callback)
    # 初始位置
    x0 = 220  # 230
    y0 = 10
    z0 = 160
    ##参照物（二维码）在机械臂基坐标系下的坐标,齐次形式,使用标定板时，该值固定
    ob2armvector = np.array([[220], [0], [0], [1]])
    # 到达初始位置
    result = move_to_target_xyz_client(x0, y0, z0)  # 控制机械臂到达初始位置
    print("motion_result: ", result)
    # 手眼标定结果
#   cam_to_end = np.array(
#                [[5.67287200e-03, -1.02631783e+00, 2.53102611e-02, 4.99652202e+01],
#      		 [-1.01078979e+00, 5.17084543e-04, -7.03614349e-02, 4.02954785e+00],
#        	 [9.21304502e-02, -6.31950711e-02, -9.59500884e-01, 3.15290844e+01],
#     	 	 [0., 0., 0., 1.00000000e+00]])
    cam_to_end = np.loadtxt("/home/U-arm/xf_arm_ws/src/arm_control/scripts/eye_hand_calib_result.txt",delimiter=',')

    if location_x == 0 and location_y == 0 and location_z == 0:
        pass
    else:
        # 物体在相机坐标下坐标
        object_in_cam = np.array([[location_x], [location_y], [location_z], [1]])
        # 获取物体在末端下的坐标
        object_in_end = np.dot(cam_to_end, object_in_cam)
        # 获取当前末端位置
        position = get_current_xyz_client()
        end_to_base = RobotMatrix(position[0], position[1], position[2])
        result_rx=np.dot(end_to_base, object_in_end)
        # 获取物体在基座下位置
        xa = result_rx[0:][0]
        ya = result_rx[0:][1]
        za = result_rx[0:][2]

        print(xa, ya, za)	
        # 控制机械臂到达指定位置
        result = move_to_target_xyz_client(xa, ya, 10)
        print(result)
