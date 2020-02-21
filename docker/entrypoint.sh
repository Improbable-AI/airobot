#!/bin/bash
set -e

# setup ros environment
source "/root/catkin_ws/devel/setup.bash"

pip install -e /home/improbable/airobot

eval "bash"

exec "$@"
