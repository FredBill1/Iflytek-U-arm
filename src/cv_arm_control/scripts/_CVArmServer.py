#!/usr/bin/env python3
import socket
from CVArmControl import CVArmControl
from ImgProcess import ImgProcess
import rospy


class CVArmServer:
    def __init__(self):
        self.cv_arm = CVArmControl()
        self.img_process = ImgProcess()

    def init(self):
        self.cv_arm.init()
        self.img_process.init()

    def msgParse(self, msg: str, client: socket.socket):
        if msg == "init":
            self.cv_arm.move_home()
            rospy.loginfo("获取Yolo")
            yolo = self.img_process.getYolo()
            print(yolo)
        elif msg.startswith("grab:"):
            print(msg[5:])
            self.cv_arm.move_home2()
        elif msg == "drop":
            rospy.loginfo("获取aruco")
            aurco = self.img_process.getAucro()
            print(aurco)
            client.send("done".encode())


__all__ = ["CVArmServer"]
