FROM ros:kilted
#FROM osrf/ros:humble-desktop-full
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    git \
    wget \
    vim \
    tmux \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/src
WORKDIR /home
RUN git clone https://github.com/kosmicplane/SenDiMoniProg-IMU.git

RUN echo "HOLA AMIGUITOS/ Hello little friends/ Ni Hao xiaopengyou/ Bonjour les petits amis/ Hallo kleine Freunde/ Ciao piccoli amici/ Привет маленькие друзья"

# Script de arranque
RUN echo '#!/bin/bash\n\
cd /home/src/SenDiMoniProg-IMU\n\
source /opt/ros/kilted/setup.bash\n\
[ -f install/setup.bash ] && source install/setup.bash\n\
apt-get update\n\
apt install ros-kilted-turtlesim\
echo "Done"\n\
exec bash' > /entrypoint.sh && chmod +x /entrypoint.sh

CMD ["/entrypoint.sh"]
