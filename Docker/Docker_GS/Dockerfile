FROM ros:kilted-perception-noble

# ===============================================================
# 🛰️ ROS 2 Kilted (Noble) + RViz + Gazebo + AHRS
# ===============================================================

# 🔹 Dependencias básicas
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
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y gz-harmonic && \
    rm -rf /var/lib/apt/lists/*

# 🔹 ROS–Gazebo + RViz + herramientas gráficas
RUN apt-get update && apt-get install -y \
    ros-kilted-ros-gz-bridge \
    ros-kilted-ros-gz-sim \
    ros-kilted-rviz2 \
    ros-kilted-rviz-common \
    ros-kilted-rviz-default-plugins \
    ros-kilted-rviz-rendering \
    ros-kilted-rviz-assimp-vendor \
    ros-kilted-rviz-resource-interfaces \
    ros-kilted-sensor-msgs-py \
    ros-kilted-geometry-msgs \
    ros-kilted-rclpy \
    ros-kilted-tf2-tools \
    ros-kilted-tf-transformations \
    ros-kilted-rqt \
    ros-kilted-rqt-common-plugins \
    netcat-traditional \
    iputils-ping \
    net-tools \
    iproute2 \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

# 🔹 Librerías Python científicas y AHRS
RUN pip3 install --no-cache-dir --break-system-packages \
    numpy scipy matplotlib pandas openpyxl ahrs websockets


RUN curl -fsSL https://tailscale.com/install.sh | sh
# ===============================================================
# 🔹 Clonar y almacenar el proyecto
# ===============================================================
# Clone project repository at build time
RUN git clone --depth 1 https://github.com/kosmicplane/SenDiMoniProg-IMU.git /opt/repo
#RUN git pull
# Copy repo to /home inside image (avoid overwriting with host volume)
RUN mkdir -p /home/SenDiMoniProg-IMU && \
    cp -a /opt/repo/. /home/SenDiMoniProg-IMU/ && \
    rm -rf /opt/repo

# ===============================================================
# 🔹 Variables de entorno ROS/Gazebo/FastDDS
# ===============================================================
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=42
ENV FASTDDS_DEFAULT_PROFILES_FILE=/home/SenDiMoniProg-IMU/DOCKER/DOCGUI/faster_profile.xml
ENV GAZEBO_MASTER_URI=http://192.168.55.1:11345
ENV GAZEBO_RESOURCE_PATH=/usr/share/gz
ENV GZ_SIM_RESOURCE_PATH=/usr/share/gz
ENV ROS_LOCALHOST_ONLY=0

# ===============================================================
# 🔹 Entrypoint mejorado
# ===============================================================
RUN echo '#!/bin/bash\n\
source /opt/ros/kilted/setup.bash\n\
echo "🚀 ROS 2 + RViz + AHRS listo para funcionar sin Internet."\n\
exec bash' > /entrypoint.sh && chmod +x /entrypoint.sh

CMD ["/entrypoint.sh"]
