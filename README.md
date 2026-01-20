# SenDiMoniProg-IMU // BADKITTEN REPOSITORY
<img width="1024" height="1024" alt="image" src="https://github.com/user-attachments/assets/a24ba357-d96c-4aaa-84e7-0d5be01b11c9" />

The SenDiMoniProg-IMU project is part of the research activities carried out within the SenDiMoniProg Laboratory of the Tracking research group. Its main objective is to integrate Inertial Measurement Units (IMUs) with embedded systems and the ROS 2 framework, enabling real-time acquisition, processing, and visualization of motion and orientation data.

IMUs provide critical information such as acceleration, angular velocity, and orientation, which are fundamental in fields like robotics, aerospace engineering, autonomous vehicles, and diagnostics. However, using IMUs in isolation has limitations due to sensor drift, noise, and calibration requirements. This project addresses these challenges by providing tools and frameworks for sensor fusion and by integrating IMU data with other subsystems such as GPS, control algorithms, and visualization platforms.

## 🚀 Prerequisites

Before working with this repository, make sure you have: 

- [Git](https://git-scm.com/)  
- A [GitHub](https://github.com/) account  
- SSH access configured on your machine  
- [Python 3.10+](https://www.python.org/downloads/)  
- [ROS 2 Kilted](https://docs.ros.org/en/kilted/Installation.html) (recommended) (ignore this for now cause the image includes everything you will need)
- [colcon](https://colcon.readthedocs.io/en/released/) for building ROS 2 workspaces  

Verify your installations:

```bash
git --version
python3 --version
ros2 --version
```

---

## 📥 Clone the Repository

Clone the repo using SSH:

```bash
git clone git@github.com:kosmicplane/SenDiMoniProg-IMU.git
cd SenDiMoniProg-IMU
```

If you cloned using HTTPS and want to switch to SSH:

```bash
git remote set-url origin git@github.com:kosmicplane/SenDiMoniProg-IMU.git
```

---

## ⚙️ Setup the Environment

1. Create and activate a Python virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate
```

2. Install Python dependencies:

```bash
pip install -r requirements.txt
```

3. Initialize rosdep (for ROS dependencies):

```bash
sudo rosdep init   # only if not initialized before
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
## 📌 Git Workflow

Always keep your repository updated:

```bash
git checkout main
git pull origin main
```

When contributing:

```bash
git checkout -b feature/your-feature-name
git add .
git commit -m "Add feature: description"
git push origin feature/your-feature-name
```

Open a **Pull Request (PR)** on GitHub once your feature is ready.

---

## ✅ Confirm Your Participation

After cloning and configuring the repo, make your first commit:

```bash
git add .
git commit -m "I have joined the repo"
git push
```

---

## 📚 Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)  
- [Git Best Practices](https://nvie.com/posts/a-successful-git-branching-model/)  
- [Colcon Build System](https://colcon.readthedocs.io/en/released/)  
- [Connecting to GitHub with SSH](https://docs.github.com/en/authentication/connecting-to-github-with-ssh)

---

## 📷 MQTT RealSense Streaming + UI

This repo includes a Jetson publisher and a desktop UI that streams and visualizes **color + depth** frames over MQTT.

### ✅ Broker configuration (LAN recommended)

Set the broker using environment variables (no hardcoded hosts):

- `MQTT_HOST`
- `MQTT_PORT`
- `MQTT_USER`
- `MQTT_PASS`

Recommended broker on the laptop (LAN):

```bash
sudo apt install mosquitto mosquitto-clients
```

### ✅ Topics

**Publisher output**

- `cam/jetson01/color_mat` — binary frame (header + raw BGR8 bytes)
- `cam/jetson01/depth_mat` — binary frame (header + raw Z16 bytes)
- `cam/jetson01/meta` — JSON metadata (`seq`, `t_cap_ns`, fps, intrinsics, model)
- `cam/jetson01/calib` — JSON calibration snapshot (`intrinsics`, `depth_scale`)
- `cam/jetson01/status` — JSON status/config snapshot

**IMU input (separate node)**

- `imu/jetson01/raw` — CSV IMU stream (accel/gyro/mag + env)

**Control input**

- `cam/jetson01/control` — JSON config payload (e.g. streaming enable/disable).

Example control payload:

```json
{
  "streaming_enabled": true
}
```

### Frame binary format (RSF1)

Frames are raw numpy matrices with a fixed little-endian header (no JPG/PNG):

```
magic:      4 bytes  b'RSF1'
kind:       uint8    (0=color, 1=depth)
seq:        uint32
t_cap_ns:   uint64   (time.time_ns() at capture)
w:          uint16
h:          uint16
channels:   uint8    (3 for color, 1 for depth)
dtype_code: uint8    (1=uint8, 2=uint16)
payload_len:uint32
payload:    raw contiguous bytes
```

### How to run (Jetson + Laptop)

**Jetson (publisher)**

```bash
export MQTT_HOST=192.168.1.10
export MQTT_PORT=1883
python3 MQTT/Mosquitto_RealSense_Camara.py
```

**Laptop (viewer)**

```bash
sudo apt install python3-pyqt6 python3-pyqt6.qtcharts
export MQTT_HOST=192.168.1.10
export MQTT_PORT=1883
python3 MQTT/qt_viewer_app.py
```

### Troubleshooting

- **No frames?** Ensure the broker is reachable in LAN and the topics match.
- **PyQt6 missing?** Install via APT: `sudo apt install python3-pyqt6 python3-pyqt6.qtcharts`.
- **Optional LZ4:** install `sudo apt install python3-lz4` and set `RSF_LZ4=1` on both sides.

### Demo Mode (no hardware)

If you do not have a RealSense available, the UI can run in **demo mode** to display synthetic frames:

```bash
DEMO_MODE=1 python3 MQTT/qt_viewer_app.py
```

### Dependencies (APT)

```bash
sudo apt install python3-paho-mqtt python3-numpy python3-opencv
```

For the UI, add Qt (if not already installed):

```bash
sudo apt install python3-pyqt6 python3-pyqt6.qtcharts
```

For the Jetson publisher, install **librealsense / pyrealsense2** for your platform.
