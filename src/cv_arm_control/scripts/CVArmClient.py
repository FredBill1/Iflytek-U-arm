#!/usr/bin/env python3

import rospy
import socket

commands = ("init", "grab:", "drop")


def main():
    rospy.init_node("CVArmClient", anonymous=True)
    HOST = rospy.get_param("~host", "192.168.1.76")
    PORT = rospy.get_param("~port", 39394)
    client = socket.socket()
    client.connect((HOST, PORT))
    while True:
        for i, c in enumerate(commands):
            print(f"{i}:{c}")
        cmd = input(">>")
        if cmd == "q":
            break
        try:
            client.send(commands[int(cmd)].encode())
        except:
            break
    client.close()


if __name__ == "__main__":
    main()
