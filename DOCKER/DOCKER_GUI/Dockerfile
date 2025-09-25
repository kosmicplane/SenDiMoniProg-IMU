FROM ros:kilted-perception-noble

# Dependencias ROS 2 + herramientas básicas
RUN apt-get update && apt-get install -y \
    lsb-release gnupg2 curl wget \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    vim \
    tmux \
    mesa-utils \
    mesa-vulkan-drivers \
    vulkan-tools \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libegl-mesa0 \
    libgbm1 \
    libglfw3 \
    libxrandr2 \
    && rm -rf /var/lib/apt/lists/*

# 🔹 Repositorio de Gazebo (Ignition/Harmonic)
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL http://packages.osrfoundation.org/gazebo.gpg | gpg --dearmor -o /etc/apt/keyrings/gazebo.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y gz-harmonic

# 🔹 Paquetes ROS–Gazebo y mensajes necesarios
RUN apt-get update && apt-get install -y \
    ros-kilted-ros-gz-bridge \
    ros-kilted-ros-gz-sim \
    ros-kilted-rviz-default-plugins \
    ros-kilted-sensor-msgs-py \
    ros-kilted-geometry-msgs \
    ros-kilted-rclpy \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home
RUN git clone https://github.com/kosmicplane/SenDiMoniProg-IMU.git

# Variables persistentes
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=42
ENV export FASTDDS_DEFAULT_PROFILES_FILE=/home/SenDiMoniProg-IMU/DOCKER/DOCGUI/faster_profile.xml 
ENV GAZEBO_MASTER_URI=http://192.168.55.1:11345
ENV GAZEBO_RESOURCE_PATH=/usr/share/gz
ENV GZ_SIM_RESOURCE_PATH=/usr/share/gz

# Entrypoint mejorado
RUN echo '#!/bin/bash\n\
source /opt/ros/kilted/setup.bash\n\
echo "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"\n\
gz gui -s MinimalScene -s WorldControl -s WorldStats -s TopicViewer\n\
exec bash' > /entrypoint.sh && chmod +x /entrypoint.sh

CMD ["/entrypoint.sh"]
