#!/usr/bin/env python3

import rospy
from ImgProcess import ImgProcess
import cv2

rospy.init_node("testimg", anonymous=True)
imgProcess = ImgProcess()
rospy.on_shutdown(imgProcess.on_shutdown)
imgProcess.init()
while not rospy.is_shutdown():
    # input(".")
    input("yolo")
    res = imgProcess.getYolo()
    print(res)
    input("aruco")
    res = imgProcess.getAucro()
    print(res)
rospy.spin()
