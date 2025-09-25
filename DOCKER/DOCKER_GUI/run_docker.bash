#!/bin/bash
xhost +local:root

export XAUTH=/tmp/.docker.xauth
export DISPLAY=${DISPLAY:-:0}
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run -it --rm \
    --device /dev/dri:/dev/dri \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume=$XAUTH:$XAUTH \
    godkitten:latest
