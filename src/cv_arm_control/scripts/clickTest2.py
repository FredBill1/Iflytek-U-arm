#!/usr/bin/env python3
import rospy
from CVArmControl import CVArmControl
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
import numpy as np

WINNAME = "ClickOnImg"


class Main:
    def init(self) -> None:
        self.arm = CVArmControl()
        self.arm.init()

        self.img = np.zeros((480, 640, 3), np.uint8)
        self.img_sub = rospy.Subscriber("/usb_cam/image_rect_color", Image, self.msgCb)
        self.cvbridge = CvBridge()
        cv2.namedWindow(WINNAME)
        cv2.setMouseCallback(WINNAME, self.clickCb)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("Init done. Click on image to pick a position.")

    def msgCb(self, src: Image) -> None:
        try:
            self.img = self.cvbridge.imgmsg_to_cv2(src, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

    def spin(self):
        while not rospy.is_shutdown():
            cv2.imshow(WINNAME, self.img)
            if cv2.waitKey(5) & 0xFF == ord("q"):
                break
        rospy.signal_shutdown("pressed 'q'")

    def clickCb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            rospy.loginfo("\nclick: x:%d y:%d" % (x, y))
            self.arm.move2d(x, y, 0.22, 0, 150)
            self.arm.use(False)
            rospy.sleep(0.3)
            # self.arm.move_xyz_relative(0, 0, 30, 50)
            # rospy.sleep(5)
            self.arm.move_home2(200)
            rospy.sleep(1)
            self.arm.use(True)

    def on_shutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("quit.")


def main():
    rospy.init_node("clickTest", anonymous=True)
    m = Main()
    m.init()
    m.arm.move_home2()
    m.arm.use(True)
    m.spin()


if __name__ == "__main__":
    main()
