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

WINNAME = "ClickOnImg"

ARUCO_PARAM = cv2.aruco.DICT_6X6_250
ARUCO_ID = 31

GOAL_STATUS = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST",
}


def line_intersection(line1, line2) -> Tuple[float, float]:
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        raise Exception("lines do not intersect")

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y


class Notifier:
    def __init__(self, state=False):
        self.cond = Condition()
        self.state = state

    def wait(self, newState=False):
        self.cond.acquire()
        self.cond.wait_for(lambda: self.state)
        self.state = newState
        self.cond.release()

    def notify(self, newState=True):
        self.cond.acquire()
        self.state = newState
        self.cond.notify()
        self.cond.release()


class ImgProcess:
    def __init__(self) -> None:
        self.arucoDict = cv2.aruco.Dictionary_get(ARUCO_PARAM)
        self.cvbridge = CvBridge()
        self.newImgNotifier = Notifier()
        self.dispNotifier = Notifier()

    def init(self) -> None:
        rospy.loginfo("ImgProcess - 等待图片话题...")
        img: Image = rospy.wait_for_message("/usb_cam/image_rect_color", Image)
        self.img = self.cvbridge.imgmsg_to_cv2(img, "bgr8")
        self.img_sub = rospy.Subscriber("/usb_cam/image_rect_color", Image, self.imgCb)
        self.yolo_img_sub = rospy.Subscriber("/darknet_ros/detection_image", Image, self.yoloImgCb)
        self.yolo_client = actionlib.SimpleActionClient("/darknet_ros/check_for_objects", CheckForObjectsAction)

        rospy.loginfo("ImgProcess - 等待darknet actionlib服务端...")
        self.yolo_client.wait_for_server()
        self.res_img = np.zeros_like(self.img)
        self.running = True
        self.dispThread = Thread(target=self.displayThread)
        self.dispThread.start()

        rospy.loginfo("ImgProcess - 进行Yolo测试...")
        self.getYolo()
        rospy.loginfo("ImgProcess - 进行Aucro测试...")
        self.getAucro()

        rospy.loginfo("ImgProcess - 初始化完成")
        rospy.on_shutdown(self.on_shutdown)

    def on_shutdown(self):
        self.newImgNotifier.notify()
        self.dispNotifier.notify()
        self.running = False
        self.dispThread.join()
        cv2.destroyAllWindows()

    def displayThread(self):
        while self.running:
            self.dispNotifier.wait()
            disp = np.concatenate((self.img, self.res_img), 1)
            cv2.imshow("img", disp)
            cv2.waitKey(1)

    def yoloImgCb(self, data: Image):
        try:
            self.res_img = self.cvbridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        self.dispNotifier.notify()

    def imgCb(self, data: Image) -> None:
        try:
            img = self.cvbridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        self.img = cv2.flip(img, 1)
        self.newImgNotifier.notify()
        self.dispNotifier.notify()

    def getAucro(self) -> Optional[Tuple[float, float]]:
        res = None
        self.newImgNotifier.wait()
        self.res_img = self.img.copy()
        gray = cv2.cvtColor(self.res_img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.arucoDict)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(self.res_img, corners, ids)
            for i, id in enumerate(ids):
                if id[0] == ARUCO_ID:
                    corner = corners[i][0]
                    res = line_intersection((corner[0], corner[2]), (corner[1], corner[3]))
                    cv2.circle(self.res_img, tuple(round(v) for v in res), 5, (0, 0, 255), 5)
                    break
        self.dispNotifier.notify()
        return res

    def getYolo(self) -> Dict[str, List[Tuple[float, float]]]:
        self.newImgNotifier.wait()
        try:
            imgMsg = self.cvbridge.cv2_to_imgmsg(self.img, "rgb8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return {}
        goal = CheckForObjectsGoal(image=imgMsg)
        while self.running:
            status = self.yolo_client.send_goal_and_wait(goal, rospy.Duration(5))
            if status == GoalStatus.SUCCEEDED:
                break
            rospy.logerr(f"ImgProcess - getYolo失败: {GOAL_STATUS[status]}")
        ret = {}
        bounding_boxes: List[BoundingBox] = self.yolo_client.get_result().bounding_boxes.bounding_boxes
        if bounding_boxes is not None:
            for box in bounding_boxes:
                if box.Class not in ret:
                    ret[box.Class] = []
                mid = ((box.xmin + box.xmax) >> 1, (box.ymin + box.ymax) >> 1)
                ret[box.Class].append(mid)
        return ret


__all__ = ["ImgProcess"]
