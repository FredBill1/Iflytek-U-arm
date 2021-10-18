#!/usr/bin/env python3
import rospy
from CVArmControl import CVArmControl
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image

WINNAME = "ClickOnImg"


class ImgBridge:
    def init(self) -> None:
        self.img_sub = rospy.Subscriber("/usb_cam/image_rect_color", Image, self.msgCb)
        self.cvbridge = CvBridge()
        cv2.namedWindow(WINNAME)
        cv2.setMouseCallback(WINNAME, self.clickCb)

    def msgCb(self, src: Image) -> None:
        try:
            img = self.cvbridge.imgmsg_to_cv2(src, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        cv2.imshow(WINNAME, img)
        cv2.waitKey(1)

    def clickCb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(x, y)


def main():
    rospy.init_node("clickTest", anonymous=True)
    # arm = CVArmControl()
    # arm.init()
    # while True:
    #     x, y = map(int, input("输入坐标: ").split())
    #     arm.move2d(x, y, 0.15)
    #     arm.use(True)
    #     arm.move_home()
    #     arm.use(False)
    imgBridge = ImgBridge()
    imgBridge.init()
    rospy.spin()


if __name__ == "__main__":
    main()
