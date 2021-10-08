#!/usr/bin/env python3

import rospy
from random import randint
from time import sleep
from .ArmControl import *


def main():
    rospy.init_node("test_arm_move")

    arm = ArmControl()
    arm.init()

    arm.move_home()

    print("home:", arm.get_pos())

    while True:
        i = 0
        while i < 10:
            x, y, z = (randint(0, 250) for _ in range(3))
            if arm.check_xyz(x, y, z):
                print(x, y, z)
                arm.move_xyz(x, y, z)
                i += 1
        arm.move_home()
        print("sleep\n")
        sleep(3)


if __name__ == "__main__":
    main()
