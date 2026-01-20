# IMU WebSocket Bridge Integration

This folder contains **non-destructive additions** for the IMU WebSocket
bridge. No existing files are modified; everything here can be copied
on top of your current `SenDiMoniProg-IMU` repository.

## 1. Python dependency (Jetson + PC containers)

Both the Jetson and PC ROS 2 Docker images need the Python package
`websockets`.

### 1.1. Jetson Dockerfile

In your Jetson Dockerfile (for example `DOCKER/Dockerfile`), locate
the Python dependencies section where `ahrs`, `numpy`, etc. are
installed. It should look similar to::

    RUN pip3 install --break-system-packages numpy scipy matplotlib ahrs pandas openpyxl

Extend that line so that it also installs ``websockets``::

    RUN pip3 install --break-system-packages numpy scipy matplotlib ahrs pandas openpyxl websockets

### 1.2. PC Dockerfile (DOCGUI)

In your PC / GUI Dockerfile (for example `DOCKER/DOCGUI/Dockerfile`),
locate the Python dependencies section where ``ahrs`` and scientific
libraries are installed. It should look similar to::

    RUN pip3 install --no-cache-dir --break-system-packages \
        numpy scipy matplotlib pandas openpyxl ahrs

Extend that line to also include ``websockets``::

    RUN pip3 install --no-cache-dir --break-system-packages \
        numpy scipy matplotlib pandas openpyxl ahrs websockets

Rebuild your images after this change.


## 2. New ROS 2 packages

Two new ROS 2 Python packages are provided under::

    ROS2/ROS2_PACKAGES/imu_ws_server
    ROS2/ROS2_PACKAGES/imu_ws_client

### 2.1. imu_ws_server (Jetson)

- **Runs on:** Jetson Nano
- **Subscribes to:** `/imu/data` (published by `imu_bt_publisher`)
- **Opens WebSocket:** `ws://0.0.0.0:8765` by default
- **Sends:** each IMU message as a JSON string to all connected clients.

Build and run on the Jetson container:

```bash
cd /home/SenDiMoniProg-IMU/ROS2/ROS2_PACKAGES
colcon build --symlink-install --packages-select imu_bt_publisher imu_ws_server
source install/setup.bash

# Terminal 1: IMU publisher (already existing)
ros2 run imu_bt_publisher imu_publisher

# Terminal 2: WebSocket server
ros2 run imu_ws_server imu_ws_server_node
```

### 2.2. imu_ws_client (PC)

- **Runs on:** PC
- **Connects to:** `ws://<JETSON_IP_OR_VPN>:8765`
- **Publishes:** `/imu/data` on the PC as `sensor_msgs/Imu`.

Build and run on the PC container:

```bash
cd /home/SenDiMoniProg-IMU/ROS2/ROS2_PACKAGES
colcon build --symlink-install --packages-select imu_ws_client
source install/setup.bash

# Replace <JETSON_IP_OR_VPN> with the reachable IP/hostname of the Jetson
ros2 run imu_ws_client imu_ws_client_node --ros-args -p ws_url:=ws://<JETSON_IP_OR_VPN>:8765
```

You can then visualize the IMU data on the PC side using standard ROS 2
tools (RViz2, Foxglove Studio, etc.) by subscribing to `/imu/data`.


## 3. Standalone test client (non-ROS)

Under the `bridge/` folder there is a simple, standalone client:

```text
bridge/test_imu_ws_client.py
```

It allows you to debug the WebSocket server without involving ROS 2.

Usage from your PC (with Python 3 and `websockets` installed)::

    cd bridge
    python3 test_imu_ws_client.py ws://<JETSON_IP_OR_VPN>:8765

You should see JSON IMU messages printed on the console. This confirms
that:

- the IMU publisher is running on the Jetson,
- the WebSocket bridge server is running, and
- your network connectivity (LAN, VPN overlay, etc.) is working.


## 4. Non-destructive overlay

This archive is meant to be extracted on top of your existing
`SenDiMoniProg-IMU` repository **without deleting or modifying** your
current files. Only new files are added:

- `ROS2/ROS2_PACKAGES/imu_ws_server/**`
- `ROS2/ROS2_PACKAGES/imu_ws_client/**`
- `bridge/test_imu_ws_client.py`
- `DOCKER/BRIDGE_INSTRUCTIONS.md`

The only manual edits required are the small additions of `websockets`
in your Dockerfiles, as described above.
