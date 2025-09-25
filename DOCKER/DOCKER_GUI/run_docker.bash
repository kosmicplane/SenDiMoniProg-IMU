#!/bin/bash
xhost +local:root

export XAUTH=/tmp/.docker.xauth
export DISPLAY=${DISPLAY:-:0}
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
export XDG_RUNTIME_DIR=/tmp/runtime-$USER
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

docker run -it --rm \
    --device=/dev/dri/card1 \
    --device=/dev/dri/renderD128 \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume=$XAUTH:$XAUTH \
    godkitten:latest
