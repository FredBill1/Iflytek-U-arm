#!/usr/bin/env python3

import rospy
import socket

commands = ("init", "grab:", "drop")


def main():
    rospy.init_node("CVArmClient", anonymous=True)
    HOST = rospy.get_param("~host", "192.168.1.76")
    PORT = rospy.get_param("~port", 39394)
    while True:
        for i, c in enumerate(commands):
            print(f"{i}:{c}")
        cmd = input(">>")
        if cmd == "q":
            break
        with socket.socket() as client:
            client.connect((HOST, PORT))
            client.send(commands[int(cmd)].encode())


if __name__ == "__main__":
    main()
