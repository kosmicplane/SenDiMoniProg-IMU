# 🐳 Docker Compose Setup for ROS 2 Kilted

This project now uses **Docker Compose** to build and run an offline-ready image for the **SenDiMoniProg-IMU** project.

The container is designed to run with host networking, GUI support, and access to `/dev/rfcomm0`. The image is built **online** (requires internet), saved, and later executed **offline** without pulling or rebuilding.

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

---

## 🛠️ Build the offline image (ONLINE)

From the `DOCKER` directory:

```bash
docker compose -f docker-compose.build.yml build
```

Save the resulting image for transfer to offline machines:

```bash
docker save badkitten:offline -o badkitten_offline.tar
```

---

## 📦 Load the image (OFFLINE)

On the offline machine, load the exported tarball:

```bash
docker load -i badkitten_offline.tar
```

---

## ▶️ Run the container (OFFLINE)

Use Docker Compose (no pull/build during runtime):

```bash
docker compose -f docker-compose.offline.yml up
```

The Compose file mirrors the previous `docker run` flags:
- `privileged: true`
- `devices: /dev/rfcomm0:/dev/rfcomm0`
- `network_mode: host` (default; see profiles below)
- GUI-friendly environment variables and X11 mounts
- `pull_policy: "never"` to avoid online pulls at runtime
- `stdin_open: true` and `tty: true` for interactive shells

### Optional profiles
- **dev**: mounts the host repo into `/home/SenDiMoniProg-IMU` for live development.

  ```bash
  docker compose -f docker-compose.offline.yml --profile dev up
  ```

- **offline_strict**: disables networking entirely (`network_mode: none`).

  ```bash
  docker compose -f docker-compose.offline.yml --profile offline_strict up
  ```

### Legacy helper script
`run_docker.bash` now wraps the offline Compose file for convenience:

```bash
./run_docker.bash
```

---

## 🚀 Inside the Container

Once inside, source ROS 2 and the workspace:

```bash
source /opt/ros/kilted/setup.bash
[ -f /home/SenDiMoniProg-IMU/install/setup.bash ] && source /home/SenDiMoniProg-IMU/install/setup.bash
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
