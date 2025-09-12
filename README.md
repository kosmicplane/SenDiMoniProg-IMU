# SenDiMoniProg-IMU

Repository for the **SenDiMoniProg-IMU** project, focused on integrating IMU sensors into embedded systems and supporting research activities within the **SenDiMoniProg** laboratory of the **Tracking** research group.  

This repository aims to simplify setup, development, and collaboration among project members.

---

## 🚀 Prerequisites

Before getting started, make sure you have the following installed on your system:

- [Git](https://git-scm.com/)
- A [GitHub](https://github.com/) account
- SSH access configured on your machine

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

In the folder where you want to store the project:

```bash
git clone git@github.com:kosmicplane/SenDiMoniProg-IMU.git
cd SenDiMoniProg-IMU
```

---

## 🔄 Configure Remote

If you need to update the repository remote:

```bash
git remote remove origin
git remote add origin git@github.com:kosmicplane/SenDiMoniProg-IMU.git
```

---

## 📌 Basic Workflow

1. Make sure you are on the `main` branch:

```bash
git checkout main
```

2. Pull the latest changes:

```bash
git pull origin main
```

3. Add your changes:

```bash
git add .
git commit -m "Description of your changes"
git push
```

---

## ✅ Confirm Your Participation

Once the repository is cloned and configured, you can make your first commit to confirm that you joined:

```bash
git add .
git commit -m "I have joined the repo"
git push
```

---

## 📚 Additional Resources

- [Official Git Documentation](https://git-scm.com/doc)  
- [Connecting to GitHub with SSH](https://docs.github.com/en/authentication/connecting-to-github-with-ssh)
