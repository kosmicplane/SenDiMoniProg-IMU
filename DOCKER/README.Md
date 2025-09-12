# 🐳 Dockerizing ROS 2 Kilted for SenDiMoniProg-IMU

This project includes a **Dockerized environment** to simplify the setup and execution of ROS 2 **Kilted** with all required dependencies for the **SenDiMoniProg-IMU** project.  

By using Docker, contributors can ensure that they all run the same environment, avoiding version conflicts and simplifying installation across different machines.

---

## 🎯 Why Dockerize ROS 2 Kilted?

ROS 2 is complex and highly dependent on the operating system. Setting it up manually can lead to problems such as:  
- Dependency conflicts between ROS distributions.  
- Python/colcon build issues.  
- Inconsistent environments across different contributors’ machines.  

Docker solves this by:  
- Packaging ROS 2 Kilted, dependencies, and the repo in one container.  
- Providing reproducibility: *“it works on my machine”* becomes *“it works everywhere”*.  
- Enabling quick rebuilds and clean environments.  
- Allowing GUI applications (RViz, Gazebo, rqt) to run via X11 forwarding.  

---

## ⚙️ Install Docker

### On Ubuntu

Update your system and install Docker:

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg lsb-release
```

Add Docker’s official GPG key:

```bash
sudo mkdir -m 0755 -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
```

Set up the repository:

```bash
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

Install Docker Engine and Compose:

```bash
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Add your user to the `docker` group so you don’t need `sudo`:

```bash
sudo usermod -aG docker $USER
newgrp docker
```

Verify the installation:

```bash
docker --version
docker run hello-world
```

---

## 🛠️ Build the Docker Image

Navigate to the `DOCKER` folder in this repo:

```bash
cd DOCKER
```

Build the image:

```bash
docker build -t ros-kilted-imu:latest .
```

---

## ▶️ Run the Container

To run interactively with GUI support (for RViz, Gazebo, etc.):

```bash
xhost local:root
XAUTH=/tmp/.docker.xauth

docker run -it --rm \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --privileged \
    ros-kilted-imu:latest
```

This will open a container with ROS 2 Kilted and your repo ready.

---

## 🧩 Using Docker Compose

Alternatively, you can use the `docker-compose.yml` file:

```bash
docker-compose up --build
```

This simplifies the process of starting the container with the proper environment.

---

## 🚀 Inside the Container

Once inside, source ROS 2 and your workspace:

```bash
source /opt/ros/kilted/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
```

Build your workspace:

```bash
colcon build --symlink-install
```

Run your IMU node:

```bash
ros2 run imu_pkg imu_node
```

Or launch a complete setup:

```bash
ros2 launch imu_pkg imu_launch.py
```

---

## 📚 References

- [Install Docker on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)  
- [ROS 2 Documentation](https://docs.ros.org/en/kilted/)  
- [Docker & ROS Integration](https://wiki.ros.org/Docker)
