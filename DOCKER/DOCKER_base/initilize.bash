# ---------------------------------------------------------
# Create an entrypoint script that initializes ROS2 and runs your node
# ---------------------------------------------------------
echo -e '#!/bin/bash
set -e
source /opt/ros/kilted/setup.bash
cd /home/SenDiMoniProg-IMU/ROS2_PACKAGES/imu_bt_publisher/imu_bt_publisher
python3 imu_publisher.py &
exec bash\n' > /entrypoint.sh && chmod +x /entrypoint.sh