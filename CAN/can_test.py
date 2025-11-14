import can
import time
import os
import subprocess

# Configuración del bus CAN
CAN_INTERFACE = 'can0'
BITRATE = 125000

def reset_can():
    """Resetea el bus CAN si entra en bus-off"""
    print("[INFO] Reiniciando bus CAN...")
    subprocess.run(f"sudo ip link set {CAN_INTERFACE} down", shell=True)
    time.sleep(0.1)
    subprocess.run(f"sudo ip link set {CAN_INTERFACE} up type can bitrate {BITRATE}", shell=True)
    time.sleep(0.1)
    print("[INFO] Bus CAN reiniciado")

# Inicialización del bus
def init_can():
    os.system(f"sudo ip link set {CAN_INTERFACE} down")
    os.system(f"sudo ip link set {CAN_INTERFACE} up type can bitrate {BITRATE}")
    bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')
    return bus

bus = init_can()

print("[INFO] Escuchando mensajes CAN...")

while True:
    try:
        message = bus.recv(timeout=1.0)  # Espera 1 segundo por mensaje
        if message:
            print(f"Mensaje recibido: ID=0x{message.arbitration_id:X}, Datos={message.data}")
        
        # Ejemplo: enviar un mensaje cada 2 segundos
        data_to_send = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]
        msg = can.Message(arbitration_id=0x123, data=data_to_send, is_extended_id=False)
        bus.send(msg)
        print(f"Mensaje enviado: ID=0x{msg.arbitration_id:X}, Datos={msg.data}")
        time.sleep(2)

    except can.CanError as e:
        print("[ERROR] CAN error:", e)
        reset_can()

    except KeyboardInterrupt:
        print("Terminando ejecución...")
        break

