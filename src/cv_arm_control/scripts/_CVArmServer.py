#!/usr/bin/env python3
import socket
from CVArmControl import CVArmControl
from ImgProcess import ImgProcess
import rospy
from typing import Dict, List, Tuple

CATAGORY = (
    "fruit",
    "beverage",
    "meat",
    "vegetable",
)

Z1 = 0.18
Z2 = 0.22
GRAB_Z = 5.0
GRAB_VEL = 50.0


class CVArmServer:
    def __init__(self):
        self.cv_arm = CVArmControl()
        self.img_process = ImgProcess()
        self.yolo: Dict[str, List[Tuple[float]]] = {}

    def init(self):
        self.cv_arm.init()
        self.img_process.init()

    def msgParse(self, msg: str, client: socket.socket):
        if msg == "init":
            self.cv_arm.move_home()
            rospy.loginfo("获取Yolo")
            for catagory, li in self.img_process.getYolo().items():
                self.yolo[catagory] = []
                for x, y in li:
                    self.yolo[catagory].append(self.cv_arm.calc2d(x, y, Z1)[:2])
            rospy.loginfo("Yolo获取完成")
            rospy.loginfo(self.yolo)

        elif msg.startswith("grab"):
            self.target = msg.split()[1]
            rospy.loginfo(f"抓取{self.target}")
            if self.target not in self.yolo or not self.yolo[self.target]:
                rospy.logerr(f"未检测到{self.target}")
            else:
                x, y = self.yolo[self.target].pop()
                self.cv_arm.move_xyz(x, y, GRAB_Z, GRAB_VEL)
                self.cv_arm.use(True)
            self.cv_arm.move_home()
            self.cv_arm.move_home2()

        elif msg == "drop":
            rospy.loginfo("获取aruco")
            aurco = self.img_process.getAucro()
            if aurco is None:
                rospy.logerr("未检测到aurco")
                # TODO
            # x, y = aurco
            # print(aurco)
            # client.send("done".encode())
            self.cv_arm.use(False)
            rospy.sleep(0.5)
            self.cv_arm.move_home()


__all__ = ["CVArmServer"]
