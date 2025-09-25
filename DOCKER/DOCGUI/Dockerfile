FROM ros:kilted-perception-noble

# Dependencias ROS y utilidades
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
    && rm -rf /var/lib/apt/lists/*

# 🔹 Añadir repositorio de Gazebo (Ignition / Harmonic)
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL http://packages.osrfoundation.org/gazebo.gpg | gpg --dearmor -o /etc/apt/keyrings/gazebo.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y gz-harmonic

WORKDIR /home
RUN git clone https://github.com/kosmicplane/SenDiMoniProg-IMU.git

# Entrypoint para GUI
RUN echo '#!/bin/bash\n\
source /opt/ros/kilted/setup.bash\n\
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n\
export GZ_SIM_RESOURCE_PATH=/usr/share/gz\n\
echo "Starting Gazebo GUI and ready for ROS 2 nodes..."\n\
exec gz gui' > /entrypoint.sh && chmod +x /entrypoint.sh

RUN export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
    export ROS_DOMAIN_ID=42 && \
    export FASTDDS_DEFAULT_PROFILES_FILE=/home/SenDiMoniProg-IMU/DOCKER/DOCGUI/faster_profile.xml && \
    export GAZEBO_MASTER_URI=http://192.168.55.1:11345\
    export GAZEBO_RESOURCE_PATH=/usr/share/gz && \
    export GZ_SIM_RESOURCE_PATH=/usr/share/gz
CMD ["/entrypoint.sh"]
