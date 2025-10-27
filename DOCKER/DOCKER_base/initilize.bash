# ---------------------------------------------------------
# Create an entrypoint script that initializes ROS2 and runs your node
# ---------------------------------------------------------
SHELL ["/bin/bash","-c"]
RUN echo -e '#!/bin/bash\n\
set -e\n\
source /opt/ros/kilted/setup.bash\n\
cd /home/SenDiMoniProg-IMU/ROS2_PACKAGES/imu_bt_publisher/imu_bt_publisher\n\
python3 imu_publisher.py &\n\
exec bash\n' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
