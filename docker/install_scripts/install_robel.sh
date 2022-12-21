#!/bin/bash

set -euxo pipefail
# mkdir -p /root/.mujoco
# wget https://www.roboti.us/download/mujoco200_linux.zip -O mujoco.zip
# unzip mujoco.zip -d /root/.mujoco
# mv /root/.mujoco/mujoco200_linux /root/.mujoco/mujoco200
# rm mujoco.zip
# wget https://www.roboti.us/file/mjkey.txt -O /root/.mujoco/mjkey.txt

# apt-get update
# DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends libosmesa6-dev libgl1-mesa-glx libglfw3 patchelf
# apt-get clean

# rm -rf /var/lib/apt/lists/*

# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/.mujoco/mujoco200/bin

pip install git+https://github.com/ROBOTIS-GIT/DynamixelSDK.git#subdirectory=python
# pip install robel

