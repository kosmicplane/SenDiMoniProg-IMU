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