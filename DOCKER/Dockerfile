FROM ros:kilted-perception-noble
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    git \
    wget \
    vim \
    tmux \
    ros-kilted-rviz-default-plugins\
    && rm -rf /var/lib/apt/lists/*


RUN mkdir -p /home/src
WORKDIR /home
RUN git clone http://github.com/kosmicplane/SenDiMoniProg-IMU.git

RUN echo "HOLA AMIGUITOS/ Hello little friends/ Ni Hao xiaopengyou/ Bonjour les petits amis/ Hallo kleine Freunde/ Ciao piccoli amici/ Привет маленькие друзья"


# (Optional) Add Gazebo repository + install Gazebo Harmonic
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://packages.osrfoundation.org/gazebo.gpg | gpg --dearmor -o /etc/apt/keyrings/gazebo.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y gz-harmonic

RUN apt-get update && apt-get install -y \
    mesa-utils \
    re\
    mesa-vulkan-drivers \
    ros-kilted-rclpy \
    python3-serial\
    vulkan-tools \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libegl-mesa0 \
    libgbm1 \
    netcat-traditional\
    iputils-ping\
    net-tools\
    iproute2\
    ros-kilted-sensor-msgs-py \
    ros-kilted-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y \
    ros-kilted-ros-gz-bridge \
    ros-kilted-ros-gz-sim \
    && rm -rf /var/lib/apt/lists/*\
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp&&\
    export ROS_DOMAIN_ID=42&&\
    export FASTDDS_DEFAULT_PROFILES_FILE=/home/SenDiMoniProg-IMU/DOCKER/DOCGUI/faster_profile.xml&&\
    export GAZEBO_MASTER_URI=http://192.168.55.1:11345\
    export GAZEBO_RESOURCE_PATH=/usr/share/gz&&\
    export GZ_SIM_RESOURCE_PATH=/usr/share/gz
RUN pip3 install --break-system-packages numpy scipy matplotlib ahrs pandas openpyxl
# Script de arranque
RUN echo '#!/bin/bash\n\
cd ..\
source /opt/ros/kilted/setup.bash\n\
[ -f install/setup.bash ] && source install/setup.bash\n\
apt-get update\n\
echo "Done"\n\
exec bash' > /entrypoint.sh && chmod +x /entrypoint.sh
CMD ["/entrypoint.sh"]
