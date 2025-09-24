xhost local:root
#docker-compose -f docker-compose.yml up -d --build

XAUTH=/tmp/.docker.xauth

docker run -it --rm \
    --device=/dev/ttyUSB0:/dev/ttyUSB0 \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --gpus all\
    --privileged \
    badkitten:latest

