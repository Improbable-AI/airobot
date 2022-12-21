#!/bin/bash

set -euxo pipefail

# git clone https://github.com/Improbable-AI/Azure_Kinect_ROS_Driver.git
git clone -b boost https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
git clone -b pr-support-rational-polynomial-distortion https://github.com/ros-planning/moveit_calibration.git
git clone https://github.com/pal-robotics/aruco_ros.git
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..
cd ..
apt update
apt-get install ros-noetic-ddynamic-reconfigure
rosdep update
rosdep install --from-paths src/Universal_Robots_Client_Library --from-paths src/moveit_calibration --from-paths src/ur5e --ignore-src -y
catkin build
