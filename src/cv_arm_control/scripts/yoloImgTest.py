#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from threading import Condition, Thread
from typing import List, Tuple, Dict, Optional
import actionlib
from darknet_ros_msgs.msg import CheckForObjectsAction, CheckForObjectsGoal, CheckForObjectsResult, BoundingBox
from actionlib_msgs.msg import GoalStatus
import numpy as np


rospy.init_node("yolo_img_test")

cvbridge = CvBridge()

img = cv2.imread("/home/fredbill/workspace/roscar/Iflytek-U-arm/src/cv_arm_control/test.jpg")
yolo_client = actionlib.SimpleActionClient("/darknet_ros/check_for_objects", CheckForObjectsAction)
yolo_client.wait_for_server()
msg = cvbridge.cv2_to_imgmsg(img, "rgb8")
goal = CheckForObjectsGoal(image=msg)
yolo_client.send_goal(goal)
data = rospy.wait_for_message("/darknet_ros/detection_image", Image)
ret = {}
bounding_boxes: List[BoundingBox] = yolo_client.get_result().bounding_boxes.bounding_boxes
if bounding_boxes is not None:
    for box in bounding_boxes:
        if box.Class not in ret:
            ret[box.Class] = []
        mid = ((box.xmin + box.xmax) >> 1, (box.ymin + box.ymax) >> 1)
        ret[box.Class].append(mid)
print(ret)
res_img = cvbridge.imgmsg_to_cv2(data, "bgr8")
cv2.imshow("yoloImg", res_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
