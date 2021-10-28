#!/usr/bin/env python3

import rospy
import socket

HOST = "localhost"
PORT = 39394

rospy.init_node("test_socket_client", anonymous=True)
client = socket.socket()

client.connect((HOST, PORT))
while True:
    cmd = input(">>").strip()
    if len(cmd) == 0:
        continue
    if cmd == "q":
        break
    client.send(cmd.encode())
    # cmd_res = client.recv(1024)
    # print(cmd_res.decode())

client.close()
