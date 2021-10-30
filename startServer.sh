#!/bin/bash

gnome-terminal --tab  -- bash -c "roslaunch cv_arm_control imgService.launch"; sleep 3

gnome-terminal --tab -- bash -c "roslaunch arm_control arm_server.launch"; sleep 3

rosrun cv_arm_control CVArmServer.py