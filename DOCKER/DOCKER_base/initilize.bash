# ---------------------------------------------------------
# Create an entrypoint script that initializes ROS2 and runs your node
# ---------------------------------------------------------
RUN echo '#!/bin/bash\n\
cd ROS2_PACKAGES/imu_bt_publisher/imu_bt_publisher\n\
python3 imu_publisher.py\n\
exec bash' > /entrypoint.sh && chmod +x /entrypoint.sh
