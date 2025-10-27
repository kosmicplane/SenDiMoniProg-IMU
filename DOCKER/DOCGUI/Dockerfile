# ===============================================================
# 🛰️ ROS 2 Kilted (Noble) + RViz + IMU Data Logger
# ===============================================================

FROM ros:kilted-perception-noble

# ---------------------------------------------------------------
# 1. Dependencies
# ---------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    lsb-release gnupg2 curl wget python3-pip \
    python3-colcon-common-extensions git vim tmux \
    mesa-utils mesa-vulkan-drivers vulkan-tools \
    libgl1-mesa-dri libglx-mesa0 libegl-mesa0 libgbm1 \
    libglfw3 libxrandr2 && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------
# 2. Gazebo
# ---------------------------------------------------------------
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL http://packages.osrfoundation.org/gazebo.gpg | \
    gpg --dearmor -o /etc/apt/keyrings/gazebo.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y gz-harmonic

# ---------------------------------------------------------------
# 3. ROS–Gazebo + RViz
# ---------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-kilted-ros-gz-bridge ros-kilted-ros-gz-sim \
    ros-kilted-rviz2 ros-kilted-rviz-common ros-kilted-rviz-default-plugins \
    ros-kilted-rviz-rendering ros-kilted-rviz-assimp-vendor \
    ros-kilted-rviz-resource-interfaces \
    ros-kilted-sensor-msgs-py ros-kilted-geometry-msgs ros-kilted-rclpy \
    ros-kilted-rqt ros-kilted-rqt-common-plugins \
    netcat-traditional iputils-ping net-tools iproute2 python3-serial && \
    rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------
# 4. Python libs
# ---------------------------------------------------------------
RUN pip3 install --break-system-packages numpy scipy matplotlib ahrs pandas openpyxl

# ---------------------------------------------------------------
# 5. Clone project
# ---------------------------------------------------------------
WORKDIR /home
RUN git clone https://github.com/kosmicplane/SenDiMoniProg-IMU.git && \
    cd SenDiMoniProg-IMU && git pull && \
    echo "✅ Project repository cloned and updated."

# ---------------------------------------------------------------
# 6. Environment
# ---------------------------------------------------------------
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION = rmw_fastrtps_cpp
# ---------------------------------------------------------------
# 7. Entrypoint
# ---------------------------------------------------------------
SHELL ["/bin/bash", "-c"]
RUN echo -e '#!/bin/bash\n\
set -e\n\
echo "🚀 Starting ROS 2 environment..."\n\
source /opt/ros/kilted/setup.bash\n\
cd /home/SenDiMoniProg-IMU/Data_Proccesing\n\
echo "📡 Launching IMU listener..."\n\
python3 imu_listener.py &\n\
sleep 3\n\
echo "🧹 Killing auto-launched Gazebo viewers..."\n\
pkill -f gz || true\n\
pkill -f \"topic viewer\" || true\n\
sleep 1\n\
echo "🧭 Starting RViz2..."\n\
rviz2 &\n\
exec bash' > /entrypoint.sh && chmod +x /entrypoint.sh
