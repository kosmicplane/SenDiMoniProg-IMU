echo -e '#!/usr/bin/env bash
set -e
source /opt/ros/kilted/setup.bash
cd /home/SenDiMoniProg-IMU/ROS2_PACKAGES/imu_bt_publisher/imu_bt_publisher
exec python3 imu_publisher.py
' > /entrypoint.sh && chmod +x /entrypoint.sh && /entrypoint.sh
