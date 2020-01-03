#!/bin/bash

set -euxo pipefail

apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key
add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u

apt-get update

apt-get install -y \
  librealsense2-dkms \
  librealsense2-utils \
  librealsense2-dev \
  librealsense2-dbg

rm -rf /var/lib/apt/lists/*
