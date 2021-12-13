#!/bin/bash

echo Waiting for arm_server...
gnome-terminal --tab  -- bash -c "roslaunch cv_arm_control clickTest.launch"; sleep 6

rosrun cv_arm_control clickTest.py