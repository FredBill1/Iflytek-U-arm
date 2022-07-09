#!/usr/bin/env python3

import rospy
import socket

commands = ("init", "grab", "drop")
category = {
    "w": "wet",
    "d": "dry",
    "r": "recycle",
    "h": "harmful",
}


def main():
    rospy.init_node("CVArmClient", anonymous=True)
    HOST = rospy.get_param("~host", "192.168.31.200")
    PORT = rospy.get_param("~port", 39394)
    while True:
        cmd = input(">>")
        if cmd == "q":
            break
        if cmd.startswith("grab"):
            t = cmd.split()[-1]
            if t in category:
                cmd = "grab " + category[t]
        with socket.socket() as client:
            client.connect((HOST, PORT))
            client.send(cmd.encode())


if __name__ == "__main__":
    main()
