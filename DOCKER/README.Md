# 🐳 Docker Compose Dev Loop for ROS 2 Kilted

This setup replaces the legacy `run_docker.bash` flow with **Docker Compose** optimized for a fast local inner-loop. The default Compose file mounts your local repository into the container so code changes are visible immediately without rebuilding. Rebuilds are only needed when system or Python dependencies change.

---

## ⚙️ Install Docker (Ubuntu only)

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg lsb-release
sudo mkdir -m 0755 -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker $USER && newgrp docker
```

---

## 🏗️ Build dependencies (first time or after dependency changes)

From the repo root:

```bash
docker compose -f DOCKER/docker-compose.yml build
```

This builds the `sendi-dev:local` image with all apt/pip dependencies but **without** copying the source code (it will be bind-mounted).

---

## 🔁 Daily development (no rebuild for code edits)

Start the container with your host repo mounted into `/home/SenDiMoniProg-IMU`:

```bash
docker compose -f DOCKER/docker-compose.yml up
```

You can also run detached and follow logs:

```bash
docker compose -f DOCKER/docker-compose.yml up -d
docker compose -f DOCKER/docker-compose.yml logs -f
```

### Optional: reproducible run without bind mount

If you need to run without mounting the host repo, use the production override (no bind mount):

```bash
docker compose -f DOCKER/docker-compose.yml -f DOCKER/docker-compose.prod.yml up
```

---

## 🖥️ X11 forwarding notes

Allow local root access (required for GUI apps inside the container):

```bash
xhost +local:root
```

Ensure an Xauthority file exists and matches the Compose configuration (`DOCKER/.env` defaults to `/tmp/.docker.xauth`):

```bash
touch /tmp/.docker.xauth
```

The Compose files already mount `/tmp/.X11-unix` and `${XAUTHORITY}` and set `DISPLAY` and `QT_X11_NO_MITSHM=1`.

---

## 🧰 Helper script (optional)

You can still use the wrapper to run the default Compose file:

```bash
./DOCKER/run_docker.bash
```

---

## 🚀 Inside the Container

Once inside, source ROS 2 and the workspace if available:

```bash
source /opt/ros/kilted/setup.bash
[ -f /home/SenDiMoniProg-IMU/install/setup.bash ] && source /home/SenDiMoniProg-IMU/install/setup.bash
```

Build your packages (symlinks avoid rebuilds on code changes):

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
