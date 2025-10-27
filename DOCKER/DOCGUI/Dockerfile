# ===============================================================
# 🛰️ ROS 2 Kilted (Noble) + RViz + IMU Data Logger
# This image launches RViz and automatically runs your IMU listener
# Located in: /home/SenDiMoniProg-IMU/Data_Proccesing/imu_listener.py
# ===============================================================

# --- Base image: ROS 2 Kilted perception stack ---
FROM ros:kilted-perception-noble

# ---------------------------------------------------------------
# 🔧 1. Basic tools and ROS 2 build dependencies
# ---------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    lsb-release gnupg2 curl wget \
    python3-pip \
    python3-colcon-common-extensions \
    git vim tmux \
    mesa-utils mesa-vulkan-drivers vulkan-tools \
    libgl1-mesa-dri libglx-mesa0 libegl-mesa0 libgbm1 \
    libglfw3 libxrandr2 \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------
# 🧩 2. Gazebo (Ignition / Harmonic)
# ---------------------------------------------------------------
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL http://packages.osrfoundation.org/gazebo.gpg | \
    gpg --dearmor -o /etc/apt/keyrings/gazebo.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y gz-harmonic

# ---------------------------------------------------------------
# 🌈 3. ROS–Gazebo bridge, RViz, and message packages
# ---------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-kilted-ros-gz-bridge \
    ros-kilted-ros-gz-sim \
    ros-kilted-rviz2 \
    ros-kilted-rviz-default-plugins \
    ros-kilted-rviz-common \
    ros-kilted-sensor-msgs-py \
    ros-kilted-geometry-msgs \
    ros-kilted-rclpy \
    ros-kilted-rqt \
    ros-kilted-rqt-common-plugins \
    netcat-traditional iputils-ping net-tools iproute2 \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------
# 🧠 4. Python libraries (ROS nodes and IMU processing)
# ---------------------------------------------------------------
RUN pip3 install --break-system-packages numpy scipy matplotlib ahrs pandas openpyxl

# ---------------------------------------------------------------
# 📦 5. Clone your project repository
# ---------------------------------------------------------------
WORKDIR /home
RUN git clone https://github.com/kosmicplane/SenDiMoniProg-IMU.git

# ---------------------------------------------------------------
# 🌍 6. Environment variables (ROS 2 & Gazebo runtime)
# ---------------------------------------------------------------
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=42
ENV FASTDDS_DEFAULT_PROFILES_FILE=/home/SenDiMoniProg-IMU/DOCKER/DOCGUI/faster_profile.xml
ENV GAZEBO_MASTER_URI=http://192.168.55.1:11345
ENV GAZEBO_RESOURCE_PATH=/usr/share/gz
ENV GZ_SIM_RESOURCE_PATH=/usr/share/gz

# ---------------------------------------------------------------
# 🚀 7. Entrypoint script
#    - Sources ROS 2 environment
#    - Launches RViz
#    - Runs the IMU listener node automatically
# ---------------------------------------------------------------
RUN echo '#!/bin/bash\n\
set -e\n\
echo "🚀 Starting ROS 2 environment..."\n\
source /opt/ros/kilted/setup.bash\n\
cd /home/SenDiMoniProg-IMU\n\
git pull\n\
cd /Data_Proccesing\n\
echo "📡 Launching IMU listener..."\n\
python3 imu_listener.py &\n\
sleep 2\n\
echo "🧭 Starting RViz2..."\n\
rviz2\n\
exec bash' > /entrypoint.sh && chmod +x /entrypoint.sh

# ---------------------------------------------------------------
# 🧾 8. Default command
# ---------------------------------------------------------------
CMD ["/entrypoint.sh"]
