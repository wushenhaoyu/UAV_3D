#!/bin/bash

# 定义launch文件路径
LAUNCH_FILE="2024.launch"

# 定义ROS包名称（根据实际情况修改）
ROS_PACKAGE="tutorial_all"

source /opt/ros/noetic/setup.bash
source /home/flmg/UAV_3D/devel/setup.bash

# 检查ROS环境是否已初始化
if [ -z "$ROS_ROOT" ]; then
    echo "ROS environment not initialized. Please source your ROS setup file (e.g., source /opt/ros/noetic/setup.bash)."
    exit 1
fi

# 启动roslaunch
echo "Starting roslaunch for ${ROS_PACKAGE}/${LAUNCH_FILE}..."
roslaunch $ROS_PACKAGE $LAUNCH_FILE

# 检查roslaunch是否成功
if [ $? -eq 0 ]; then
    echo "roslaunch started successfully."
else
    echo "Failed to start roslaunch. Please check your launch file and ROS environment."
fi
