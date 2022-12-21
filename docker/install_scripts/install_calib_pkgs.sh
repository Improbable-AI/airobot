#!/bin/bash

cd /root
apt-get update 
mkdir -p calib_ws/src; cd calib_ws/src
git clone -b pr-support-rational-polynomial-distortion https://github.com/ros-planning/moveit_calibration.git
ln -s /usr/local/lib/python3.8/dist-packages/numpy/core/include/numpy /usr/include/numpy
cd ../
source /opt/ros/noetic/setup.bash
apt update
rosdep install -y --from-paths src/. --ignore-src --rosdistro noetic
catkin build
