# NVIDIA Jetson Nano SSH Connection Setup

This document explains how to connect to a Jetson Nano using **SSH**.  
There are two main communication methods:

1. **USB + SSH**  
2. **Wi-Fi (LAN) + SSH**  

> ⚠️ This guide is written for **Linux/Ubuntu** environments.  
> On Windows, steps may differ (e.g., using PuTTY or PowerShell).

---

## 1. USB + SSH Connection

This method requires connecting the Jetson Nano to your PC with a USB cable.

1. **Install SSH on your system (if not already installed):**  
   ```bash
   sudo apt install openssh-client -y

    Connect to the Jetson Nano from your PC:

    ssh badkitten@192.168.55.1

    On the first connection, type yes to accept and save the fingerprint.

    Enter the password for user badkitten (provided by the lab or previously set on your Jetson Nano).

✅ Done! You are now connected through USB.
Remember: this method only works with a physical USB connection.
2. Wi-Fi (LAN) + SSH Connection

This method requires both your Jetson Nano and your PC to be on the same Wi-Fi network.

    Find the Jetson Nano’s IP address
    On the Jetson Nano terminal, run:

ip -4 addr show wlP1p1s0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}'
nmcli -g IP4.ADDRESS device show wlP1p1s0 | cut -d/ -f1
hostname -I

This will return the Wi-Fi interface IP address (e.g., 192.168.137.151).

Connect to the Jetson Nano with password authentication:

ssh badkitten@192.168.137.151

(IP will depend on the command output above).

Connect to the Jetson Nano with public key (no password):

a. Generate an SSH key pair on your PC (if you don’t already have one):

ssh-keygen -t ed25519 -C "badkitten@jetson"

b. Copy your public key to the Jetson Nano:

ssh-copy-id -i ~/.ssh/id_ed25519.pub badkitten@192.168.137.151
# Setup de conexión con NVIDIA Jetson Nano

Este documento describe los pasos para conectarse a un Jetson Nano utilizando **SSH**.  
Actualmente existen dos métodos principales de comunicación:

1. **USB + SSH**  
2. **Wi-Fi (LAN) + SSH**  

> ⚠️ Este instructivo está diseñado para un entorno **Linux/Ubuntu**.  
> En Windows los pasos pueden variar (ej. uso de PuTTY o PowerShell).

---

## 1. Conexión vía USB + SSH

Este método requiere conectar el Jetson Nano a tu PC mediante un cable USB.

1. **Instala SSH** en tu sistema (si aún no lo tienes):  
   ```bash
   sudo apt install openssh-client -y

    Conéctate al Jetson Nano desde tu PC:

    ssh badkitten@192.168.55.1

    La primera vez que te conectes, confirma el fingerprint escribiendo yes.

    Introduce la contraseña del usuario badkitten (proporcionada por el laboratorio o definida en tu Jetson Nano).

✅ ¡Listo! Ya estarás dentro del sistema a través de USB.
Recuerda que este método solo funciona cuando hay conexión física por USB.
2. Conexión vía Wi-Fi (LAN) + SSH

Este método requiere que tanto el Jetson Nano como tu PC estén conectados a la misma red Wi-Fi.

    Detecta la IP del Jetson Nano
    En la terminal del Jetson Nano ejecuta:

ip -4 addr show wlP1p1s0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}'
nmcli -g IP4.ADDRESS device show wlP1p1s0 | cut -d/ -f1
hostname -I

Esto devolverá la dirección IP de la interfaz Wi-Fi (ejemplo: 192.168.137.151).

Conéctate al Jetson Nano usando contraseña:

ssh badkitten@192.168.137.151

(La IP dependerá de la salida del comando anterior).

Conéctate al Jetson Nano usando clave pública (sin contraseña):

a. Genera un par de llaves en tu PC (si no lo tienes ya):

ssh-keygen -t ed25519 -C "badkitten@jetson"

b. Copia tu llave pública al Jetson Nano:

ssh-copy-id -i ~/.ssh/id_ed25519.pub badkitten@192.168.137.151

c. Asegura permisos en el Jetson Nano:

chmod 700 ~/.ssh
chmod 600 ~/.ssh/authorized_keys
chown -R badkitten:badkitten ~/.ssh

d. Conéctate al Jetson Nano sin contraseña:

    ssh badkitten@192.168.137.151

3. Configuración recomendada de seguridad (opcional)

Cuando ya confirmes que el acceso con llaves SSH funciona, es recomendable deshabilitar el acceso por contraseña para reforzar la seguridad.

    Edita el archivo /etc/ssh/sshd_config en el Jetson Nano:

PasswordAuthentication no
PubkeyAuthentication yes
KbdInteractiveAuthentication no
UsePAM yes
PermitRootLogin prohibit-password

Reinicia el servicio SSH:

sudo systemctl restart ssh
c. Ensure correct permissions on the Jetson Nano:

chmod 700 ~/.ssh
chmod 600 ~/.ssh/authorized_keys
chown -R badkitten:badkitten ~/.ssh

d. Connect without password:

    ssh badkitten@192.168.137.151

3. Recommended Security Configuration (Optional)

Once public key authentication is confirmed working, it’s best to disable password login for better security.

    Edit the /etc/ssh/sshd_config file on the Jetson Nano:

PasswordAuthentication no
PubkeyAuthentication yes
KbdInteractiveAuthentication no
UsePAM yes
PermitRootLogin prohibit-password

Restart the SSH service:

    sudo systemctl restart ssh

Now, only key-based authentication will be allowed, blocking brute-force password attacks.
4. Final Notes

    On Linux, this setup works out-of-the-box.

    On Windows, use a client like PuTTY or the built-in ssh in PowerShell.

    For additional security:

        Consider changing the default SSH port (22).

        Enable and configure a firewall (e.g., UFW) with specific rules.




VNS:

first connect ur pc ussing SSH WIRELESS To jetsson nano and after that:


1. Jetson:

badkitten@badkitten:~$ x11vnc -display :0 -auth guess -forever -rfbauth ~/.vnc/passwd -listen 0.0.0.0        -xkb -noxrecord -noxfixes -noxdamage -permitfiletransfer -capslock -repeat -shared 

2. IN UR pc power shell:
    *Windows Download VNC viewer

    *linux:
    sudo apt update
    sudo apt install tigervnc-viewer remmina -y
    sudo ufw allow 5900/tcp
    vncviewer 192.168.137.151:5900


