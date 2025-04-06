#!/bin/bash             
cd ~/Desktop/zip/ros_libraries_ws
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
exit 0
