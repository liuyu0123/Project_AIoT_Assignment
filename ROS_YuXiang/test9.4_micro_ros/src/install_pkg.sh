#!/bin/bash
cd "$(dirname "$0")" || exit

# 
git clone https://github.com/micro-ROS/micro-ROS-Agent.git -b $ROS_DISTRO
git clone https://github.com/micro-ROS/micro_ros_msgs.git -b $ROS_DISTRO
git clone https://github.com/fishros/ros_serial2wifi.git
git clone https://github.com/fishros/ydlidar_ros2.git -b fishbot