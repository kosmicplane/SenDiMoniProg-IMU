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
    --privileged \
    -p 11345:11345 \
    -p 11346:11346 \
    badkitten:latest

# ==============================
# ROS 2 + Gazebo en Jetson Nano
# ==============================

#  middleware
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Domin
export ROS_DOMAIN_ID=42

#  (FastDDS)
export FASTDDS_DEFAULT_PROFILES_FILE=/home/SenDiMoniProg-IMU/DOCKER/DOCGUI/faster_profile.xml

# Gazebo (servidor en Jetson)
export GAZEBO_MASTER_URI=http://192.168.55.1:11345
export GAZEBO_RESOURCE_PATH=/usr/share/gz
export GZ_SIM_RESOURCE_PATH=/usr/share/gz

echo "bad kitten on board"