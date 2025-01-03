#!/usr/bin/env python3
import socket
from CVArmControl import CVArmControl
from ImgProcess import ImgProcess
import rospy
from rospy.core import is_shutdown_requested
from typing import Dict, List, Tuple
from random import randint
from math import atan2


Z1 = 0.16
Z2 = 0.21
GRAB_Z = 10.0
GRAB_VEL = 50.0
DROP_Z = -5.0
DROP_VEL = 150.0

GET_YOLO_EVERY_TIME = False
REPEAT_GRAB = False

STD_POS = (350, 200)


class CVArmServer:
    def __init__(self):
        self.cv_arm = CVArmControl()
        self.img_process = ImgProcess()
        self.yolo: Dict[str, List[Tuple[float]]] = {}
        self.target = ""
        self.apriltag: Tuple[float, float] = tuple()

    def init(self) -> None:
        self.cv_arm.init()
        self.img_process.init()
        self.prepare()

    def home(self) -> None:
        self.cv_arm.use(False)
        self.cv_arm.move_home()

    def ready(self) -> None:
        self.cv_arm.move_home(80)
        self.cv_arm.move_home2()

    def checkTarget(self) -> bool:
        if self.target not in self.yolo or not self.yolo[self.target]:
            rospy.logerr(f"未检测到{self.target}，或已经抓完所有这类的卡片")
            return False
        return True

    def grab(self) -> None:
        rospy.loginfo(f"抓取{self.target}")
        x, y = self.yolo[self.target].pop()
        self.cv_arm.move_xyz(x, y, GRAB_Z, GRAB_VEL)
        self.cv_arm.use(True)

    def getYolo(self):
        rospy.loginfo("获取Yolo")
        self.yolo = {}
        for catagory, li in self.img_process.getYolo().items():
            self.yolo[catagory] = []
            for x, y in li:
                self.yolo[catagory].append(self.cv_arm.calc2d(x, y, Z1)[:2])
        rospy.loginfo("Yolo获取完成")
        rospy.loginfo(self.yolo)

    def prepare(self):
        self.apriltag = tuple()
        self.home()
        self.getYolo()
    
    @staticmethod
    def send_client(client, msg: str):
        rospy.loginfo(f"发送到客户端: {msg}")
        client.send(msg.encode())

    def msgParse(self, msg: str, client: socket.socket):
        if msg == "init":
            self.prepare()

        elif msg.startswith("grab"):
            self.target = msg.split()[1]
            if self.checkTarget():
                self.grab()
            self.ready()

        elif msg == "drop":
            while True:
                if is_shutdown_requested():
                    return
                if not REPEAT_GRAB or not self.apriltag:
                    while True:
                        if is_shutdown_requested():
                            return
                        rospy.loginfo("获取apriltag")
                        apriltag = self.img_process.getApriltag()
                        cnt = 0
                        while apriltag is None:
                            if is_shutdown_requested():
                                return
                            cnt += 1
                            if cnt == 6:
                                self.cv_arm.move_home2()
                                cnt = 0
                            else:
                                self.cv_arm.move_xyz_relative(randint(-30, 30), randint(-30, 30), 0)
                            rospy.logerr("未检测到apriltag")
                            apriltag = self.img_process.getApriltag()
                            # TODO 一段时间都没检测到

                        x, y = apriltag
                        if self.cv_arm.move2d(x, y, Z2, DROP_Z, DROP_VEL):
                            self.apriltag = (x, y)
                            break
                        # 位置超限
                        rospy.logerr("移动到apriltag坐标失败")
                        dx, dy = STD_POS[0] - x, y - STD_POS[1]
                        yaw = atan2(dy, dx)
                        self.send_client(client, f"move {yaw}")
                        return

                else:
                    x, y = self.apriltag
                    self.cv_arm.move2d(x, y, Z2, DROP_Z, DROP_VEL)

                self.cv_arm.use(False)
                rospy.sleep(0.3)
                
                if not REPEAT_GRAB or not self.checkTarget():
                    self.send_client(client, "done")
                    break
                rospy.sleep(0.2)
                self.home()
                if GET_YOLO_EVERY_TIME:
                    rospy.sleep(0.2)
                    self.getYolo()
                    if not self.checkTarget():
                        self.send_client(client, "done")
                        break
                self.grab()
                self.ready()
            rospy.sleep(0.2)
            self.home()


__all__ = ["CVArmServer"]
