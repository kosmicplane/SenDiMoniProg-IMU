import time
import os

# Todos los GPIO disponibles que quieras testear
gpio_list = list(range(316, 512))  # según el rango de tus gpiochips

# Exportar y configurar como input todos los GPIO si no lo están
for gpio in gpio_list:
    if not os.path.exists(f"/sys/class/gpio/gpio{gpio}"):
        os.system(f"echo {gpio} | sudo tee /sys/class/gpio/export")
    os.system(f"echo in | sudo tee /sys/class/gpio/gpio{gpio}/direction")

print("Toca o conecta el pin físico del INT y se detectará el GPIO correspondiente...")

# Leer continuamente hasta detectar cambio
last_values = {}
for gpio in gpio_list:
    with open(f"/sys/class/gpio/gpio{gpio}/value", 'r') as f:
        last_values[gpio] = f.read().strip()

try:
    while True:
        for gpio in gpio_list:
            with open(f"/sys/class/gpio/gpio{gpio}/value", 'r') as f:
                val = f.read().strip()
            if val != last_values[gpio]:
                print(f"GPIO detectado: {gpio}, cambio de {last_values[gpio]} -> {val}")
                last_values[gpio] = val
        time.sleep(0.05)  # 50 ms

except KeyboardInterrupt:
    print("Saliendo...")
