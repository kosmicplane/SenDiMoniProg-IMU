# SenDiMoniProg — Embedded Sensing, ROS 2 & Navigation Interfaces

<p align="center">
  <strong>IMU · GNSS · Intel RealSense · ROS 2 · Jetson-class computing · ESP32 telemetry · MQTT / WebSocket transport</strong>
</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/images/research/ncu/embedded-installation.png" width="47%" alt="Embedded installation">
  <img src="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/images/research/ncu/pointcloud-map.png" width="47%" alt="RealSense point-cloud visualization">
</p>

This repository collects the embedded sensing and communication work developed within the **SenDiMoniProg Laboratory at National Central University (Taiwan)**. The objective is to move sensor data reliably from distributed hardware into a ROS 2 environment where it can be logged, visualized, synchronized, and consumed by downstream navigation and state-estimation components.

The repository focuses on the infrastructure immediately surrounding navigation:

~~~text
physical sensors
→ embedded acquisition
→ local filtering / calibration
→ network transport
→ ROS 2 topics
→ visualization / logging
→ downstream estimation and autonomy
~~~

The implemented work includes IMU acquisition, RealSense color/depth streaming, MQTT and WebSocket transport, ROS 2 bridging, containerized workflows, and laboratory / field-oriented testing.

---

## System architecture

~~~mermaid
flowchart LR
    A[IMU / GNSS / RealSense] --> B[ESP32 / Jetson acquisition]
    B --> C[Calibration + timestamping]
    C --> D[MQTT / WebSocket transport]
    D --> E[ROS 2 interfaces]
    E --> F[RViz / desktop UI / logging]
    E --> G[Navigation and estimation consumers]
~~~

### IMU bridge

The WebSocket bridge separates the embedded publisher from the workstation ROS graph:

~~~text
IMU hardware
→ ROS 2 /imu/data on Jetson
→ imu_ws_server
→ WebSocket transport
→ imu_ws_client on PC
→ ROS 2 /imu/data
→ RViz / Foxglove / logging
~~~

This design is useful when DDS discovery or direct ROS 2 communication is inconvenient across network boundaries.

---

## Camera / depth transport

The RealSense path uses MQTT to stream color and depth matrices together with timing and calibration metadata.

Representative topics:

| Topic | Content |
|---|---|
| cam/jetson01/color_mat | raw BGR8 image matrix |
| cam/jetson01/depth_mat | raw Z16 depth matrix |
| cam/jetson01/meta | sequence, capture time, FPS, intrinsics, model |
| cam/jetson01/calib | intrinsics and depth scale |
| cam/jetson01/status | status / configuration snapshot |
| imu/jetson01/raw | IMU stream |
| cam/jetson01/control | runtime control payload |

The frame header preserves the information needed to reconstruct the matrix deterministically:

~~~text
magic       4 bytes  'RSF1'
kind        uint8    0=color, 1=depth
seq         uint32
t_cap_ns    uint64
w, h        uint16
channels    uint8
dtype_code  uint8
payload_len uint32
payload     raw contiguous bytes
~~~

For timing analysis, the relevant end-to-end transport quantity is

[
t_{e2e}=t_{receive}-t_{capture}.
]

Project testing documented a reduction of the ESP32 → Jetson → server telemetry path from **92 ms to 6 ms** after pipeline optimization in the tested configuration.

---

## Sensor-model perspective

An IMU does not directly provide drift-free position. A useful measurement abstraction is

[
omega_m=omega+b_g+n_g,
]

[
a_m=R^	op(a-g)+b_a+n_a,
]

where b_g and b_a represent sensor biases and n_g and n_a measurement noise. This is why calibration, frame conventions, timestamp consistency, and synchronized transport matter before any downstream estimator is evaluated.

This repository primarily documents the **measurement, transport, and ROS-interface layers**. Downstream estimator performance should be validated separately from the communication layer that feeds it.

---

## Visual evidence

<table>
<tr>
<td width="33%" align="center">
<a href="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/media/projects/ncu-depth-pointcloud.mp4">
<img src="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/images/research/ncu/pointcloud-map.png" width="100%" alt="Depth point cloud">
</a><br><b>Depth / point-cloud pipeline</b>
</td>
<td width="33%" align="center">
<a href="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/media/projects/ncu-rviz-demo.mp4">
<img src="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/images/research/ncu/realsense.webp" width="100%" alt="ROS 2 RViz demo">
</a><br><b>ROS 2 / RViz integration</b>
</td>
<td width="33%" align="center">
<img src="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/images/research/ncu/hardware.webp" width="100%" alt="Embedded hardware"><br><b>Embedded hardware</b>
</td>
</tr>
</table>

<p align="center">
  <img src="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/images/research/ncu/iipp-conference.png" width="31%" alt="IIPP conference">
  <img src="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/images/research/ncu/team-prof-pan.png" width="31%" alt="Research team with Prof. Min-Chun Pan">
  <img src="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/images/research/ncu/ncu-team-sign.png" width="31%" alt="NCU research team">
</p>

Click the first two panels to open the project videos.

---

## Repository map

~~~text
Sensors/
├── IMU/               acquisition, logging, calibration-oriented work
├── Camera_RS/         RealSense image / depth transport
└── RTK_GNSS/          GNSS-oriented integration area

Bridge/
├── MQTT/              MQTT transport components
└── test_imu_ws_client.py

ROS2/                  ROS 2 packages and workspace material
Docker/                reproducible runtime environments
Oriented_Tests/        field / scenario-specific tests
Instructive/           bridge and setup instructions
~~~

---

## Running the IMU WebSocket bridge

### Jetson-side server

~~~bash
cd /home/SenDiMoniProg-IMU/ROS2/ROS2_PACKAGES
colcon build --symlink-install --packages-select imu_bt_publisher imu_ws_server
source install/setup.bash

ros2 run imu_bt_publisher imu_publisher
ros2 run imu_ws_server imu_ws_server_node
~~~

### Workstation client

~~~bash
cd /home/SenDiMoniProg-IMU/ROS2/ROS2_PACKAGES
colcon build --symlink-install --packages-select imu_ws_client
source install/setup.bash

ros2 run imu_ws_client imu_ws_client_node --ros-args \
  -p ws_url:=ws://<JETSON_IP_OR_VPN>:8765
~~~

Then inspect /imu/data using RViz2, Foxglove, ros2 topic echo, or your own consumer node.

---

## Running the RealSense MQTT path

### Broker configuration

~~~bash
export MQTT_HOST=<BROKER_IP>
export MQTT_PORT=1883
~~~

### Publisher

~~~bash
python3 MQTT/Mosquitto_RealSense_Camara.py
~~~

### Desktop viewer

~~~bash
python3 MQTT/qt_viewer_app.py
~~~

A synthetic demo mode is available when no camera is connected:

~~~bash
DEMO_MODE=1 python3 MQTT/qt_viewer_app.py
~~~

---

## Engineering notes

- Preserve sensor timestamps as close to acquisition as possible.
- Keep coordinate frames explicit at every interface.
- Treat network transport and DDS discovery as measurable parts of the sensing pipeline.
- Record calibration state and sensor provenance with datasets.
- Validate a downstream estimator separately from the transport layer that feeds it.

See [BRIDGE_INSTRUCTIONS.md](Instructive/BRIDGE_INSTRUCTIONS.md) for the detailed WebSocket overlay instructions.
