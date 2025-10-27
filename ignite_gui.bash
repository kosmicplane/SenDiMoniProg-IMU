echo '#!/bin/bash\n\
set -e\n\
echo "🚀 Starting ROS 2 environment..."\n\
source /opt/ros/kilted/setup.bash\n\
cd /home/SenDiMoniProg-IMU\n\
echo "🔄 Updating local repo..."\n\
git fetch --all && git reset --hard origin/main || true\n\
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
