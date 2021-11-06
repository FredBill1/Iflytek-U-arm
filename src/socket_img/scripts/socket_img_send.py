#!/usr/bin/env python3

import socket
import cv2
import numpy as np

HOST = "localhost"
PORT = 34567


cap = cv2.VideoCapture(0)
encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]


def loop():
    ret, frame = cap.read()
    if not ret:
        return
    img_encode = cv2.imencode(".jpg", frame, encode_param)[1]
    data = np.array(img_encode)
    lenth = str(len(data)).ljust(16)

    with socket.socket() as client:
        client.connect((HOST, PORT))
        client.send(lenth.encode())
        client.send(data)


print("开始传输...")
while True:
    loop()
