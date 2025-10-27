# 🐳 Docker Setup for ROS 2 Kilted

This project provides a Dockerized environment to run **ROS 2 Kilted** with all required dependencies for the **SenDiMoniProg-IMU** project.  

By using Docker, contributors can run the same development environment without conflicts between ROS versions, Python packages, or system dependencies.

---

## 🎯 Why Dockerize ROS 2 Kilted?

- Avoids version conflicts (ROS distributions, Python, system libraries).  
- Provides a reproducible development environment.  
- Simplifies onboarding of new contributors.  
- Allows running GUI tools (RViz, Gazebo, rqt) from inside the container.  
- Ensures everyone uses the same ROS 2 Kilted setup.  

---

## ⚙️ Install Docker (Ubuntu only)

1. Update your system and install required packages:

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg lsb-release
```

2. Add Docker’s official GPG key:

```bash
sudo mkdir -m 0755 -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
```

3. Set up the repository:

```bash
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

4. Install Docker Engine and Compose:

```bash
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

5. Add your user to the `docker` group (no need for `sudo` every time):

```bash
sudo usermod -aG docker $USER
newgrp docker
```

6. Verify installation:

```bash
docker --version
```

Expected output (your case):

```
Docker version 28.4.0, build d8eb465
```

---

## 🛠️ Build the Docker Image

From the `DOCKER` directory in this repository:

```bash
sudo docker build -t badkitten .
```

This will create an image named **badkitten** using the provided `Dockerfile`.

---

## ▶️ Run the Container

Simply use the provided script:

```bash
sudo ./run_docker.bash
```

This script runs the container with the correct settings (network, display, volumes, privileges).

---

## 🚀 Inside the Container

Once inside, source ROS 2 and the workspace:

```bash
source /opt/ros/kilted/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
```

Build your packages:

```bash
colcon build --symlink-install
```

Run your IMU node:

```bash
ros2 run imu_pkg imu_node
```

Or launch a setup:

```bash
ros2 launch imu_pkg imu_launch.py
```

---

## 📚 References

- [Docker Official Documentation](https://docs.docker.com/engine/install/ubuntu/)  
- [ROS 2 Documentation](https://docs.ros.org/en/kilted/)  
- [Colcon Build Tool](https://colcon.readthedocs.io/en/released/)

