#!/bin/bash             
cd ~/Desktop/zip/ros_libraries_ws
source devel/setup.bash
sleep 2
roslaunch fast_lio mapping_mid360.launch
exit 0
