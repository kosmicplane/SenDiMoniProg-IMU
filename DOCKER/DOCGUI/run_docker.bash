#!/bin/bash
# Permitir que Docker use tu X11
xhost +local:root

# Opcional: si usas autenticación X11
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Ejecutar el contenedor GUI
docker run -it --rm \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    godkitten:latest
# ==============================
# ROS 2 + Gazebo en PC GUI
# ==============================

#  middleware
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Domin
export ROS_DOMAIN_ID=42

#(FastDDS)
export FASTDDS_DEFAULT_PROFILES_FILE=/home/SenDiMoniProg-IMU/DOCKER/DOCGUI/faster_profile.xml

# Gazebo (se conecta al servidor en Jetson)
export GAZEBO_MASTER_URI=http://192.168.55.1:11345
export GAZEBO_RESOURCE_PATH=/usr/share/gz
export GZ_SIM_RESOURCE_PATH=/usr/share/gz
