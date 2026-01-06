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

## 🔑 SSH Key Setup

1. Generate a new SSH key (replace `"your github email"` with your GitHub email):

```bash
ssh-keygen -t ed25519 -C "your github email"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
cat ~/.ssh/id_ed25519.pub
```

2. Copy the generated public key and add it to your GitHub account:  
   - Go to **Settings > SSH and GPG keys**  
   - Click **New SSH Key**  
   - Paste your key and save

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

### ✅ Topics

**Publisher output**

- `cam/jetson01/color_jpg` — JPEG bytes (BGR) for the color camera
- `cam/jetson01/depth_jpg` — JPEG bytes (colorized depth preview)
- `cam/jetson01/depth_z16_png` — **aligned** depth as 16‑bit PNG (Z16)
- `cam/jetson01/meta` — JSON metadata (`seq`, `t_wall`, sizes, fps, intrinsics)
- `cam/jetson01/calib` — JSON calibration snapshot (`intrinsics`, `depth_scale`)
- `cam/jetson01/status` — JSON status/config snapshot

**Control input**

- `cam/jetson01/control` — JSON config payload to change JPEG quality, FPS, resolution, and depth publishing **live**.

Example control payload:

```json
{
  "jpeg_quality": 20,
  "pub_hz": 15,
  "color_w": 424,
  "color_h": 240,
  "publish_depth_preview": true,
  "publish_depth_raw": true
}
```

### Message Formats

**Meta (`cam/jetson01/meta`)**

```json
{
  "seq": 120,
  "t_wall": 1732576301.123,
  "color_bytes": 14523,
  "depth_bytes": 12310,
  "depth_raw_bytes": 43218,
  "w": 424,
  "h": 240,
  "pub_hz": 20,
  "pub_fps": 19.8,
  "jpeg_quality": 20,
  "publish_depth_preview": true,
  "publish_depth_raw": true,
  "intrinsics": {"fx": 615.4, "fy": 615.2, "ppx": 320.1, "ppy": 240.2},
  "depth_scale": 0.001
}
```

**Status (`cam/jetson01/status`)**

```json
{
  "config": {
    "color_w": 424,
    "color_h": 240,
    "color_fps": 30,
    "depth_w": 424,
    "depth_h": 240,
    "depth_fps": 30,
    "pub_hz": 20,
    "jpeg_quality": 20,
    "publish_depth_preview": true,
    "publish_depth_raw": false
  },
  "intrinsics": {"fx": 615.4, "fy": 615.2, "ppx": 320.1, "ppy": 240.2},
  "depth_scale": 0.001,
  "t_wall": 1732576301.456
}
```

### How to Run

**Publisher (Jetson / RealSense):**

```bash
python3 MQTT/Mosquitto_RealSense_Camara.py
```

**UI (desktop):**

```bash
python3 MQTT/Mosquitto_plotter_IMUCAM.py
```

### Demo Mode (no hardware)

If you do not have a RealSense available, the UI can run in **demo mode** to display synthetic frames:

```bash
DEMO_MODE=1 python3 MQTT/Mosquitto_plotter_IMUCAM.py
```

### Dependencies

```bash
pip install paho-mqtt numpy opencv-python matplotlib pandas
```

For the Jetson publisher, install **librealsense / pyrealsense2** for your platform.

