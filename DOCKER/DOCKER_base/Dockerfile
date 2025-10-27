# ---------------------------------------------------------
# Base image: ROS2 Kilted (Noble) with perception packages
# ---------------------------------------------------------
FROM ros:kilted-perception-noble

# ---------------------------------------------------------
# Install basic dependencies and ROS2 build tools
# ---------------------------------------------------------
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

# ---------------------------------------------------------
# Clone your ROS2 project into /home
# ---------------------------------------------------------
RUN mkdir -p /home/src
WORKDIR /home
RUN git clone http://github.com/kosmicplane/SenDiMoniProg-IMU.git

# ---------------------------------------------------------
# Persistent environment variables (for ROS2 and Gazebo)
# ---------------------------------------------------------
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=42
ENV FASTDDS_DEFAULT_PROFILES_FILE=/home/SenDiMoniProg-IMU/DOCKER/DOCGUI/faster_profile.xml
ENV GAZEBO_MASTER_URI=http://192.168.55.1:11345
ENV GAZEBO_RESOURCE_PATH=/usr/share/gz
ENV GZ_SIM_RESOURCE_PATH=/usr/share/gz

# ---------------------------------------------------------
# Install Gazebo Harmonic and related drivers
# ---------------------------------------------------------
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://packages.osrfoundation.org/gazebo.gpg | gpg --dearmor -o /etc/apt/keyrings/gazebo.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y gz-harmonic

# ---------------------------------------------------------
# Install graphics, networking, and ROS2 bridge packages
# ---------------------------------------------------------
RUN apt-get update && apt-get install -y \
    mesa-utils \
    mesa-vulkan-drivers \
    ros-kilted-rclpy \
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

RUN pip3 install ahrs --break-system-packages
# ---------------------------------------------------------
# Create an entrypoint script that initializes ROS2 and runs your node
# ---------------------------------------------------------
RUN echo '#!/bin/bash\n\
echo "🚀 Starting ROS2 environment..."\n\
source /opt/ros/kilted/setup.bash\n\
cd /home/SenDiMoniProg-IMU\n\
git pull\n\
cd ROS2_PACKAGES/imu_bt_publisher/imu_bt_publisher\n\
python3 imu_publisher.py\n\
exec bash' > /entrypoint.sh && chmod +x /entrypoint.sh

# ---------------------------------------------------------
# Default command: execute the entrypoint script
# ---------------------------------------------------------
ENTRYPOINT ["/entrypoint.sh"]
