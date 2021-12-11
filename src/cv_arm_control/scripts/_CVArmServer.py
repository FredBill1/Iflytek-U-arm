#!/usr/bin/env python3
import socket
from CVArmControl import CVArmControl
from ImgProcess import ImgProcess
import rospy
from rospy.core import is_shutdown_requested
from typing import Dict, List, Tuple
from random import randint

CATAGORY = (
    "fruit",
    "beverage",
    "meat",
    "vegetable",
)

Z1 = 0.18
Z2 = 0.22
GRAB_Z = 10.0
GRAB_VEL = 50.0
DROP_Z = -5.0
DROP_VEL = 150.0

GET_YOLO_EVERY_TIME = False


class CVArmServer:
    def __init__(self):
        self.cv_arm = CVArmControl()
        self.img_process = ImgProcess()
        self.yolo: Dict[str, List[Tuple[float]]] = {}
        self.target = ""
        self.aucro: Tuple[float, float] = tuple()

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
        self.aucro = tuple()
        self.home()
        self.getYolo()

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
                if not self.aucro:
                    while True:
                        if is_shutdown_requested():
                            return
                        rospy.loginfo("获取aruco")
                        aurco = self.img_process.getAucro()
                        cnt = 0
                        while aurco is None:
                            if is_shutdown_requested():
                                return
                            cnt += 1
                            if cnt == 6:
                                self.cv_arm.move_home2()
                                cnt = 0
                            else:
                                self.cv_arm.move_xyz_relative(randint(-30, 30), randint(-30, 30), 0)
                            rospy.logerr("未检测到aurco")
                            aurco = self.img_process.getAucro()
                            # TODO 一段时间都没检测到

                        x, y = aurco
                        if self.cv_arm.move2d(x, y, Z2, DROP_Z, DROP_VEL):
                            self.aucro = (x, y)
                            break
                        rospy.logerr("移动到aurco坐标失败")
                        # TODO 位置超限
                        # client.send("fail".encode())

                else:
                    x, y = self.aucro
                    self.cv_arm.move2d(x, y, Z2, DROP_Z, DROP_VEL)

                self.cv_arm.use(False)
                rospy.sleep(0.3)
                if not self.checkTarget():
                    client.send("done".encode())
                    break
                rospy.sleep(0.2)
                self.home()
                if GET_YOLO_EVERY_TIME:
                    rospy.sleep(0.2)
                    self.getYolo()
                    if not self.checkTarget():
                        client.send("done".encode())
                        break
                self.grab()
                self.ready()
            rospy.sleep(0.2)
            self.home()


__all__ = ["CVArmServer"]
