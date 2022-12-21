#!/bin/bash
set -e

# setup ros environment
#source "/root/catkin_ws/devel/setup.bash"
source /root/.bashrc
cd /workspace

eval "bash"

exec "$@"
