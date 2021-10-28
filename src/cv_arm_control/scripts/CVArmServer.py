#!/usr/bin/env python3

import rospy
import socket
import socketserver
import threading
from _CVArmServer import CVArmServer

cvArmServer = CVArmServer()


class RequestHandler(socketserver.BaseRequestHandler):
    def handle(self):
        while True:
            msg = self.request.recv(1024)
            if not msg:
                return
            msg = msg.decode()
            rospy.loginfo("收到消息：%s", msg)
            cvArmServer.msgParse(msg, self.request)

    def setup(self):
        rospy.loginfo("连接建立: %s:%d" % self.client_address)

    def finish(self):
        rospy.loginfo("连接断开: %s:%d" % self.client_address)
        self.request.close()


def main():
    rospy.init_node("CVArmServer", anonymous=True)
    cvArmServer.init()
    HOST = rospy.get_param("~host", "192.168.1.76")
    PORT = rospy.get_param("~port", 39394)
    server = socketserver.TCPServer((HOST, PORT), RequestHandler)
    rospy.on_shutdown(lambda: rospy.loginfo("正在关闭..."))
    rospy.on_shutdown(server.shutdown)

    rospy.loginfo("开启服务端，正在等待连接...")
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.start()
    rospy.spin()
    server_thread.join()
    server.server_close()
    rospy.loginfo("关闭")


if __name__ == "__main__":
    main()
