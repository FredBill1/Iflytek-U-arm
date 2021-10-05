#!/usr/bin/env python3
# data :06.01.2020
# Author: xinnie@iflytek.com
import rospy
from arm_server_v14 import ArmServer

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("arm_server")
        AR = ArmServer()
        AR.Arm_Init()
        rospy.loginfo("arm server node is started,waiting for client...")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down arm server node.")
