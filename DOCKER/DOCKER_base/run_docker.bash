
export DISPLAY=:0
xhost +local:root
#docker-compose -f docker-compose.yml up -d --build
XAUTH=/tmp/.docker.xauth
export DISPLAY=${DISPLAY:-:0}
#!/bin/bash

USER_HOME=$(getent passwd $SUDO_USER | cut -d: -f6)
PROJECT_DIR="$USER_HOME/Desktop/SenDiMoniProg-IMU"

cd "$PROJECT_DIR"
echo "🔄 Updating repository before mounting..."
git pull origin main || git pull origin master
echo "✅ Repository updated."
sudo ufw disable
echo "ufw disabled"
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
