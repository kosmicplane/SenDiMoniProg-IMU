xhost local:root
#docker-compose -f docker-compose.yml up -d --build
XAUTH=/tmp/.docker.xauth
export DISPLAY=${DISPLAY:-:0}
docker run -it --rm \
    --device=/dev/rfcomm0:/dev/rfcomm0 \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/SenDiMoniProg-IMU/ROS2_PACKAGES:/root/ros2_ws/src" \
    --volume="$XAUTH:$XAUTH" \
    --privileged \
    badkitten:latest
