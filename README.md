# SenDiMoniProg — Embedded Sensing, ROS 2 & Navigation Interfaces

<p align="center">
  <strong>IMU · GNSS · Intel RealSense · ROS 2 · Jetson-class computing · ESP32 telemetry · MQTT / WebSocket transport</strong>
</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/images/research/ncu/embedded-installation.png" width="48%" alt="Embedded sensing installation">
  <img src="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/images/research/ncu/team-prof-pan.png" width="48%" alt="Research team with Prof. Min-Chun Pan">
</p>

This repository collects embedded sensing, transport, and ROS 2 integration work developed within the **SenDiMoniProg Laboratory at National Central University (Taiwan)**.

The engineering objective is to move sensor data from distributed hardware into a ROS 2 environment while preserving the information needed for downstream navigation: timestamps, coordinate-frame meaning, calibration state, transport provenance, and repeatable logging.

The implemented pipeline spans

~~~text
physical sensors
→ embedded acquisition
→ calibration / timestamping
→ MQTT or WebSocket transport
→ ROS 2 topics
→ RViz / desktop visualization / logging
→ downstream navigation and estimation
~~~

The repository should therefore be read primarily as a **sensor-interface and distributed robotics infrastructure project**, not as a claim that every downstream estimator or autonomy algorithm has already been fully validated.

---

## System architecture

~~~mermaid
flowchart LR
    A[IMU / GNSS / RealSense] --> B[ESP32 / Jetson acquisition]
    B --> C[Calibration + timestamping]
    C --> D[MQTT / WebSocket transport]
    D --> E[ROS 2 interfaces]
    E --> F[RViz / desktop UI / logging]
    E --> G[Navigation / estimation consumers]
~~~

### IMU bridge

The WebSocket bridge separates the embedded ROS graph from the workstation:

~~~text
IMU hardware
→ ROS 2 /imu/data on Jetson
→ imu_ws_server
→ WebSocket transport
→ imu_ws_client on workstation
→ ROS 2 /imu/data
→ RViz / Foxglove / logging
~~~

This architecture is useful when direct DDS discovery is unreliable or inconvenient across network boundaries.

---

## IMU measurement model

An inertial sensor does not directly provide drift-free position. A standard measurement abstraction is

$$
\boldsymbol{\omega}_m
=
\boldsymbol{\omega}
+
\mathbf b_g
+
\mathbf n_g,
$$

for gyroscope measurements, and

$$
\mathbf a_m
=
R^\top
\left(
\mathbf a-\mathbf g
\right)
+
\mathbf b_a
+
\mathbf n_a,
$$

for accelerometer measurements.

Here:

- $\mathbf b_g$ and $\mathbf b_a$ are sensor biases;
- $\mathbf n_g$ and $\mathbf n_a$ represent measurement noise;
- $R$ encodes the selected frame convention.

This is why calibration, timestamps, and coordinate frames are first-class parts of the pipeline rather than bookkeeping details.

---

## Camera and depth transport

The Intel RealSense path streams color/depth matrices together with metadata needed to reconstruct each frame consistently.

Representative topics include:

| Topic | Content |
|---|---|
| cam/jetson01/color_mat | raw BGR8 image matrix |
| cam/jetson01/depth_mat | raw Z16 depth matrix |
| cam/jetson01/meta | sequence, capture time, FPS, intrinsics, model |
| cam/jetson01/calib | intrinsics and depth scale |
| cam/jetson01/status | status / configuration snapshot |
| imu/jetson01/raw | IMU stream |
| cam/jetson01/control | runtime control payload |

The transport header preserves deterministic reconstruction information:

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

---

## End-to-end timing

For a captured sample, the transport latency is evaluated as

$$
t_{\mathrm{e2e}}
=
t_{\mathrm{receive}}
-
t_{\mathrm{capture}}.
$$

In the tested ESP32 → Jetson → server pipeline, optimization reduced end-to-end latency from **92 ms to 6 ms**.

This value is specific to the tested configuration and should be interpreted as a systems result for that pipeline rather than a universal ROS 2 or network benchmark.

---

## Operational evidence

### RealSense depth / point-cloud path

<p align="center">
  <a href="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/media/projects/ncu-depth-pointcloud.mp4">
    <img src="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/images/research/ncu/pointcloud-map.png" width="760" alt="RealSense depth and point-cloud visualization">
  </a>
</p>

The clip documents depth acquisition and point-cloud visualization rather than only showing the final ROS topic state.

### ROS 2 / RViz integration

<p align="center">
  <a href="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/media/projects/ncu-rviz-demo.mp4">
    <img src="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/images/research/ncu/realsense.webp" width="760" alt="ROS 2 RViz sensor integration">
  </a>
</p>

This demonstrates sensor information reaching the ROS 2 visualization layer after network transport and bridging.

### Embedded hardware

<p align="center">
  <img src="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/images/research/ncu/hardware.webp" width="760" alt="Embedded sensing hardware">
</p>

> Click either video thumbnail to open the corresponding MP4.

---

## Research environment and field context

<p align="center">
  <img src="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/images/research/ncu/iipp-conference.png" width="32%" alt="IIPP conference">
  <img src="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/images/research/ncu/ncu-team-sign.png" width="32%" alt="NCU team">
  <img src="https://raw.githubusercontent.com/kosmicplane/kosmicplane.github.io/main/assets/images/research/ncu/network-dedicated.webp" width="32%" alt="Dedicated network test configuration">
</p>

These images document the research environment and network/sensor-integration context without duplicating the hardware and point-cloud figures above.

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

The resulting <code>/imu/data</code> stream can be inspected with RViz2, Foxglove, <code>ros2 topic echo</code>, or another ROS 2 consumer.

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

## Validation boundary

The repository documents the **measurement, transport, and ROS-interface layers**. EKF/UKF fusion, VIO/SLAM, and higher-level autonomy can consume this infrastructure, but their estimator performance must be validated independently from the transport layer.

That distinction is important: reliable delivery of calibrated sensor data is necessary for navigation, but it is not itself proof of estimator accuracy.

---

## Engineering principles

- preserve sensor timestamps as close to acquisition as possible;
- keep coordinate frames explicit at every interface;
- treat transport latency and DDS/network behavior as measurable engineering quantities;
- record calibration state and sensor provenance with datasets;
- separate sensor/transport validation from downstream estimator validation;
- prefer reproducible launch, logging, and containerized workflows over ad-hoc experiment setup.

See [BRIDGE_INSTRUCTIONS.md](Instructive/BRIDGE_INSTRUCTIONS.md) for the detailed WebSocket overlay instructions.
