#!/bin/bash

gnome-terminal -t "ArmControl" -x bash -c "roslaunch arm_control arm_server.launch; exrc bash" \
--tab -t "Yolo" -x bash -c "roslaunch cv_arm_control imgService.launch; exrc bash" \
--tab t "CVArmServer" -x bash -c "rosrun cv_arm_control CVArmServer.py; exrc bash"