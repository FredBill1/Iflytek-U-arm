#!/usr/bin/env python3

import rospy
import socket
import socketserver
import threading
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from camera_info_manager import CameraInfoManager
from sensor_msgs.msg import CameraInfo

img_pub: rospy.Publisher = None
info_pub: rospy.Publisher = None
camInfo: CameraInfo = None
cvbridge = CvBridge()


def recv_size(sock: socket.socket, count: int) -> bytes:
    buf = b""
    while count:
        newbuf = sock.recv(count)
        if not newbuf:
            return None
        buf += newbuf
        count -= len(newbuf)
    return buf


class RequestHandler(socketserver.BaseRequestHandler):
    def handle(self):
        length = recv_size(self.request, 16)
        if length is None:
            return self.request.close()

        bytesData = recv_size(self.request, int(length.decode()))
        self.request.close()

        data = np.fromstring(bytesData, dtype="uint8")
        decimg = cv2.imdecode(data, 1)
        msg = cvbridge.cv2_to_imgmsg(decimg, encoding="bgr8")
        img_pub.publish(msg)
        info_pub.publish(camInfo.getCameraInfo())


if __name__ == "__main__":
    rospy.init_node("socket_img_recv")

    HOST = rospy.get_param("~host", "localhost")
    PORT = rospy.get_param("~port", 34567)
    NAMESP = rospy.get_param("~ns", "usb_cam").strip("/")
    NAME = rospy.get_param("~name", "head_camera").strip("/")
    PUB_NAME = rospy.get_param("~pub_name", "image_raw").strip("/")

    IMG_TOPIC = NAMESP + "/" + PUB_NAME
    INFO_TOPIC = NAMESP + "/camera_info"
    img_pub = rospy.Publisher(IMG_TOPIC, Image, queue_size=1)
    info_pub = rospy.Publisher(INFO_TOPIC, CameraInfo, queue_size=1, latch=True)
    server = socketserver.ThreadingTCPServer((HOST, PORT), RequestHandler)

    camInfo = CameraInfoManager(cname=NAME, namespace=NAMESP)
    camInfo.loadCameraInfo()
    info_pub.publish(camInfo.getCameraInfo())

    rospy.on_shutdown(lambda: rospy.loginfo("正在关闭..."))
    rospy.on_shutdown(server.shutdown)
    rospy.loginfo(f"服务已开启，地址为{HOST}:{PORT}，图片话题为{IMG_TOPIC}，info话题为{INFO_TOPIC}")
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.start()
    rospy.spin()
    server_thread.join()
    server.server_close()
    rospy.loginfo("关闭")
