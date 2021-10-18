#!/usr/bin/env python3
import rospy
from CVArmControl import CVArmControl


def main():
    rospy.init_node("test1")
    arm = CVArmControl()
    arm.init()
    while True:
        x, y = map(int, input("输入坐标: ").split())
        arm.move2d(x, y, 0.15)
        arm.use(True)
        arm.move_home()
        arm.use(False)
    rospy.spin()


if __name__ == "__main__":
    main()
