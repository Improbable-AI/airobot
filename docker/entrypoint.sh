#!/bin/bash
set -e

# setup ros environment
source "/root/catkin_ws/devel/setup.bash"

eval "bash"

exec "$@"
