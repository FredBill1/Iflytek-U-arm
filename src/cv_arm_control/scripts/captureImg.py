#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

DIR = "/mnt/sd/trainData"
TYPE = "meat"
DELAY = 0
COUNT = 100


class Main:
    def __init__(self):
        self.bridge = CvBridge()

    def getImg(self):
        image = rospy.wait_for_message("/usb_cam/image_rect_color", Image)
        img = self.bridge.imgmsg_to_cv2(image, "bgr8")
        return img

    def cap(self):
        d = os.path.join(DIR, TYPE)
        if not os.path.exists(d):
            s = input(d + "\n目录不存在，是否创建?(y/N): ")
            if s != "Y" and s != "y":
                return
            os.makedirs(d)
        N = len(os.listdir(d))
        for i in range(N, N + COUNT):
            if input("按回车采集, 输入q退出: ") == "q":
                break
            img = self.getImg()
            file = os.path.join(d, f"{i:05d}.jpg")
            print(file)
            cv2.imwrite(file, img)
            rospy.sleep(DELAY)
        print("done.")


def main():

    rospy.init_node("test_cv", anonymous=True)
    m = Main()
    m.cap()


if __name__ == "__main__":
    main()
