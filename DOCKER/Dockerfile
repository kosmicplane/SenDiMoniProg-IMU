FROM ros:kilted-perception-noble

# -------------------------------
# System dependencies installation
# -------------------------------
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    git \
    wget \
    vim \
    tmux \
    ros-kilted-rviz-default-plugins \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------
# Workspace base
# -------------------------------
RUN mkdir -p /home/SenDiMoniProg-IMU
WORKDIR /home/SenDiMoniProg-IMU

# -------------------------------
# Additional system packages
# -------------------------------
RUN apt-get update && apt-get install -y \
    mesa-utils \
    mesa-vulkan-drivers \
    python3-serial \
    vulkan-tools \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libegl-mesa0 \
    libgbm1 \
    netcat-traditional \
    iputils-ping \
    net-tools \
    iproute2 \
    ros-kilted-sensor-msgs-py \
    ros-kilted-geometry-msgs \
    ros-kilted-ros-gz-bridge \
    ros-kilted-ros-gz-sim \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------
# Python dependencies
# -------------------------------
RUN pip3 install --break-system-packages numpy scipy matplotlib ahrs pandas openpyxl

# -------------------------------
# ROS environment variables
# -------------------------------
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=42
ENV FASTDDS_DEFAULT_PROFILES_FILE=/home/SenDiMoniProg-IMU/DOCKER/DOCGUI/faster_profile.xml
ENV GAZEBO_MASTER_URI=http://192.168.55.1:11345
ENV GAZEBO_RESOURCE_PATH=/usr/share/gz
ENV GZ_SIM_RESOURCE_PATH=/usr/share/gz
ENV ROS_LOCALHOST_ONLY=0

# -------------------------------
# Entrypoint setup
# -------------------------------
RUN echo '#!/bin/bash\n\
source /opt/ros/kilted/setup.bash\n\
[ -f /home/SenDiMoniProg-IMU/install/setup.bash ] && source /home/SenDiMoniProg-IMU/install/setup.bash\n\
echo "🚀 ROS 2 + Gazebo + AHRS listo sin Internet"\n\
exec bash' > /entrypoint.sh && chmod +x /entrypoint.sh

CMD ["/entrypoint.sh"]
