FROM ros:kilted-perception-noble
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    git \
    wget \
    vim \
    tmux \
    ros-kilted-rviz2\
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/src
WORKDIR /home
RUN git clone https://github.com/kosmicplane/SenDiMoniProg-IMU.git

RUN echo "HOLA AMIGUITOS/ Hello little friends/ Ni Hao xiaopengyou/ Bonjour les petits amis/ Hallo kleine Freunde/ Ciao piccoli amici/ Привет маленькие друзья"


# (Optional) Add Gazebo repository + install Gazebo Harmonic
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL http://packages.osrfoundation.org/gazebo.gpg | gpg --dearmor -o /etc/apt/keyrings/gazebo.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y gz-harmonic

RUN apt-get update && apt-get install -y \
    mesa-utils \
    mesa-vulkan-drivers \
    ros-kilted-rclpy \
    python3-serial\
    vulkan-tools \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libegl-mesa0 \
    libgbm1 \
    ros-kilted-sensor-msgs-py \
    ros-kilted-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# Script de arranque
RUN echo '#!/bin/bash\n\
cd ..\
source /opt/ros/kilted/setup.bash\n\
[ -f install/setup.bash ] && source install/setup.bash\n\
apt-get update\n\
echo "Done"\n\
#pip3 install -r SenDiMoniProg-IMU/requirements.txt\n\
exec bash' > /entrypoint.sh && chmod +x /entrypoint.sh

CMD ["/entrypoint.sh"]
