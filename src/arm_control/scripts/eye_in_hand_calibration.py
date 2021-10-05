#!/usr/bin/env python3
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

location_x=0
location_y=0
location_z=0
angular_x=0
angular_y=0
angular_z=0
cameralist = []
R1Tlist = []

# 根据位置向量以及roll角得到齐次矩阵
def RobotMatrix(angle_z, px, py, pz):  # arm end
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
    global location_x, location_y, location_z, angular_x, angular_y, angular_z
    location_x = msg.linear.x
    location_y = msg.linear.y
    location_z = msg.linear.z

'''
作用:移动机械臂到指定位置
'''
def move_to_target_xyz_client(x, y, z):
    rospy.wait_for_service('/arm/move_to_target_xyz')
    try:
        move_to_target_xyz = rospy.ServiceProxy(
            '/arm/move_to_target_xyz', Move_Target_3d)  # 获得一个请求处理函数的句柄，在下面会用到
        # 直接将参数按.srv文件中的顺序填好，传入句柄，即可得到结果
        resp1 = move_to_target_xyz("xyz", x, y, z, 0, 0, 0, 0, 20, "MOVJ")
        return resp1.success
    except:
        print("error")

'''
作用:获取机械臂当前直角坐标系位置
'''
def get_current_xyz_client():
    rospy.wait_for_service('/arm/get_current_xyz')
    try:
        get_current_xyz = rospy.ServiceProxy(
            '/arm/get_current_xyz', Get_Current_3d)  # 获得一个请求处理函数的句柄，在下面会用到
        # 直接将参数按.srv文件中的顺序填好，传入句柄hh，即可得到结果
        print("get_s_next")
        resp1 = get_current_xyz("xyz")
        print("get_s")
        return resp1.x_s,resp1.y_r,resp1.z_h
    except:
        print("error")
        
'''
作用:获取当前关节１的角度
'''       
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
    rospy.init_node('eye_hand_calibration', anonymous=True)
    rospy.Subscriber("qcode_detect_result", Twist, Qcode_in_cam_callback)
    
    # 标定初始位置
    x0 = 190#190
    y0 = 0#0
    z0 = 160#160
    
    # 参照物（二维码）在机械臂基坐标系下的坐标,齐次形式,使用标定板时，该值固定
    ob2armvector = np.array([[215], [0], [0], [1]]) 
    # 控制机械臂到达初始位置　
    result = move_to_target_xyz_client(x0, y0, z0)  
    print("motion_result: ", result)

    #步长
    delta_x = 20
    delta_y = 30
    delta_z = 20
    #遍历12个点来进行数据采集
    num = 0
    for i in [0,-1,1]:
        for j in [0,-1,1]:
            for k in [0,-1,1]:
                
                print("到达第%d个位置"%num)
            
                # 确定其他位置
                x = x0 + i*delta_x
                y = y0 + j*delta_y
                z = z0 + k*delta_z

                result = move_to_target_xyz_client(x, y, z)
                print("motion_result: ", result)
                if result == 1: 
                    num += 1
                    # 获取当前末端位置
                    xyz_pos = get_current_xyz_client()
                    print('xyz_position: {}'.format(xyz_pos))

                    ret = get_current_angle()
                    print('servo_1 angle: {}'.format(ret))
                    time.sleep(1)
                
                    # 获取机械臂末端相对于基座的位姿
                    R1 = RobotMatrix(ret, xyz_pos[0], xyz_pos[1], xyz_pos[2])
                    print("RobotMatrix:",R1)
                    R1_inv = np.linalg.pinv(R1)
                    R1Tlist = np.append(R1Tlist, R1_inv * ob2armvector)
    
                    # 获取参照物在图像中的坐标并追加到列表中
                    camera_vector = np.array([location_x, location_y, location_z, 1])
                    cameralist = np.append(cameralist, camera_vector)
                    print("camera_vector",camera_vector)
                    print(camera_vector)
                else:
                    print("clibration failed,the %dst position can not reached"%num)
                    break

    print("一共有%d个有效标定点位"%num)

    # 将物体在相机坐标系下的位置列表reshape
    c = np.array(cameralist).reshape(num, 4)
    d = np.transpose(c)

    # 将物体
    R1t = np.array(R1Tlist).reshape(num, 4)  # object to end
    R1t_ = np.transpose(R1t)

    # 最小二乘求解
    C1 = np.transpose(d)
    print("   ")
    E = np.linalg.inv(np.dot(d, C1))
    D = np.dot(R1t_, C1)
    cam2end_matrix = np.dot(D, E)
    print("cam2end_matrix:\n",cam2end_matrix)
    cam2end_matrix[abs(cam2end_matrix) < 0.0001] = 0 
    np.savetxt("/home/U-arm/xf_arm_ws/src/arm_control/scripts/eye_hand_calib_result.txt",cam2end_matrix,fmt = '%f',delimiter = ',')

    # 回到初始位置
    result = move_to_target_xyz_client(x0, y0, z0)
    print("motion_result: ",result)
