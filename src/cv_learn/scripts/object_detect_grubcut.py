#!/usr/bin/env python3
import os
import rospy
import rospkg
import sys
import cv2
import time
import argparse
import warnings
import numpy as np
from sys import platform
from sensor_msgs.msg import Image as MyImage

def image_callback(image_data):
	np_arr = np.fromstring(image_data.data, np.uint8)
	img = np_arr.reshape(480,640, 3)
	rows,cols,channels = img.shape[:]

	mask = np.zeros(img.shape[:2], dtype=np.uint8)
	rect = (550,200,250,300)
	bgdmodel = np.zeros((1,65),np.float64)
	fgdmodel = np.zeros((1,65),np.float64)
	cv2.grabCut(img,mask,rect,bgdmodel,fgdmodel,5,mode=cv2.GC_INIT_WITH_RECT)
	mask2 = np.where((mask==1) + (mask==3), 255, 0).astype('uint8')
	print(mask2.shape)
	result = cv2.bitwise_and(img,img,mask=mask2) 
	cv2.imshow('video',result)#cv2.waitKey(1)
	cv2.waitKey(0)


if __name__ == '__main__':
    try:
        rospy.init_node("object_grubcut_demo")
        image_sub = rospy.Subscriber("/usb_cam/image_raw",MyImage,image_callback,queue_size=1) 
        rospy.loginfo("person_follow_node is started..")
        #cv2.waitKey(0)
        rospy.spin()
        #cv2.waitKey(0)       
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down object_grubcut_node.")
        cv2.destroyAllWindows()

