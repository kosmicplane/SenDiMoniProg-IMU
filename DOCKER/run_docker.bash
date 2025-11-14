

xhost local:root
#docker-compose -f docker-compose.yml up -d --build
XAUTH=/tmp/.docker.xauth
export DISPLAY=${DISPLAY:-:0}
#!/bin/bash

USER_HOME=$(getent passwd $SUDO_USER | cut -d: -f6)
PROJECT_DIR="$USER_HOME/Desktop/SenDiMoniProg-IMU"

cd "$PROJECT_DIR"
echo "🔄 Updating repository before mounting..."
<<<<<<< HEAD
echo "✅ Repository updated."

=======
>>>>>>> 3d478198205f01e105933bc721210b8ff1bad7b7
docker run -it --rm \
    --device=/dev/rfcomm0:/dev/rfcomm0 \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --privileged \
    goodkitten:offline
