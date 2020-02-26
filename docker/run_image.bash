IMAGE=anthonysimeonov/airobot-cuda-dev:latest
XAUTH=/tmp/.docker.xauth
CAMERA_CALIB_DIR=/root/catkin_ws/src/camera_calibration
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="$CAMERA_CALIB_DIR/:/home/improbable/camera_calibration" \
    --volume="$PWD/../:/home/improbable/airobot/" \
    --privileged \
    --runtime=nvidia \
    --net=host \
    ${IMAGE}

