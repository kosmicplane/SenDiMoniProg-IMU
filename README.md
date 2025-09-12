# SenDiMoniProg-IMU

Repository for the **SenDiMoniProg-IMU** project, focused on integrating IMU sensors with embedded systems and the **ROS 2** middleware for real-time data processing, monitoring, and research within the **SenDiMoniProg Laboratory** of the **Tracking** research group.  

This repository provides source code, configuration files, and documentation to ease development and ensure collaboration among contributors.

---

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
