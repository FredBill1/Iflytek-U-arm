#!/usr/bin/env python3

import rospy
import socket


def main():
    rospy.init_node("CVArmClient", anonymous=True)
    HOST = rospy.get_param("~host", "192.168.1.76")
    PORT = rospy.get_param("~port", 39394)
    client = socket.socket()
    client.connect()


if __name__ == "__main__":
    main()
