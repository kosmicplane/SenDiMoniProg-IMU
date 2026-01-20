#!/usr/bin/env python3
import serial
import time
import os  # no lo usamos mucho, pero lo dejo por similitud
import math

import rclpy
from sensor_msgs.msg import Imu

PORT = "/dev/rfcomm0"
BAUDRATE = 230400
PRINT_HZ = 10.0
PRINT_PERIOD = 1.0 / PRINT_HZ

# CSV fields esperados (12 valores)
FIELDS = [
    "ax_g", "ay_g", "az_g",
    "gx_dps", "gy_dps", "gz_dps",
    "mx_uT", "my_uT", "mz_uT",
    "p_hpa", "t_C", "alt_m"
]

# Conversión a SI para el mensaje Imu
G_TO_MS2 = 9.80665          # g -> m/s^2
DEG_TO_RAD = math.pi / 180  # deg/s -> rad/s


def connect_serial():
    while True:
        try:
            ser = serial.Serial(PORT, BAUDRATE, timeout=0)
            return ser
        except Exception as e:
            print(f"⚠️  Retrying connection: {e}", flush=True)
            time.sleep(1)


def parse_csv12(line: str):
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != 12:
        return None
    try:
        vals = list(map(float, parts))
        return dict(zip(FIELDS, vals))
    except ValueError:
        return None


def clear_screen():
    # más rápido que os.system("clear") y funciona bien en terminal
    print("\x1b[2J\x1b[H", end="")


def build_imu_msg(node, sample: dict) -> Imu:
    """Construye un mensaje Imu desde un dict 'sample'."""
    now = node.get_clock().now().to_msg()
    msg = Imu()
    msg.header.stamp = now
    msg.header.frame_id = "imu_link"

    # Aceleración lineal [g] -> [m/s^2]
    msg.linear_acceleration.x = sample["ax_g"] * G_TO_MS2
    msg.linear_acceleration.y = sample["ay_g"] * G_TO_MS2
    msg.linear_acceleration.z = sample["az_g"] * G_TO_MS2

    # Velocidad angular [deg/s] -> [rad/s]
    msg.angular_velocity.x = sample["gx_dps"] * DEG_TO_RAD
    msg.angular_velocity.y = sample["gy_dps"] * DEG_TO_RAD
    msg.angular_velocity.z = sample["gz_dps"] * DEG_TO_RAD

    # Orientación: identidad (no hacemos estimación)
    msg.orientation.w = 1.0
    msg.orientation.x = 0.0
    msg.orientation.y = 0.0
    msg.orientation.z = 0.0

    return msg


def main():
    # ---- ROS 2 init ----
    rclpy.init()
    node = rclpy.create_node("simple_imu_publisher")
    imu_pub = node.create_publisher(Imu, "/imu/data", 10)

    # ---- Serial setup ----
    ser = connect_serial()
    rx_buf = ""
    last_sample = None
    last_print_t = time.monotonic()

    clear_screen()
    print("✅ Connected. Ctrl+C to stop.\n", flush=True)
    print("✅ ROS2 node 'simple_imu_publisher' publishing on /imu/data\n", flush=True)

    try:
        while rclpy.ok():
            # Procesar callbacks ROS (aunque aquí casi no haya)
            rclpy.spin_once(node, timeout_sec=0.0)

            n = ser.in_waiting
            if n:
                rx_buf += ser.read(n).decode("utf-8", errors="ignore")
                while "\n" in rx_buf:
                    line, rx_buf = rx_buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    sample = parse_csv12(line)
                    if sample is not None:
                        last_sample = sample

            now = time.monotonic()
            if last_sample is not None and (now - last_print_t) >= PRINT_PERIOD:
                last_print_t = now
                s = last_sample

                # ---- imprimir en consola (igual que tu script) ----
                clear_screen()
                print("✅ ESP32 IMU (última muestra)  |  Ctrl+C para salir\n")
                print(f"ACC [g]   ax={s['ax_g']:+8.3f}  ay={s['ay_g']:+8.3f}  az={s['az_g']:+8.3f}")
                print(f"GYR [dps] gx={s['gx_dps']:+8.3f}  gy={s['gy_dps']:+8.3f}  gz={s['gz_dps']:+8.3f}")
                print(f"MAG [uT]  mx={s['mx_uT']:+9.3f}  my={s['my_uT']:+9.3f}  mz={s['mz_uT']:+9.3f}")
                print(f"P/T/Alt   P={s['p_hpa']:9.2f} hPa   T={s['t_C']:6.2f} °C   Alt={s['alt_m']:8.2f} m\n")
                print(f"Print rate: {PRINT_HZ:.1f} Hz")

                # ---- publicar en ROS 2 ----
                imu_msg = build_imu_msg(node, s)
                imu_pub.publish(imu_msg)

            if n == 0:
                time.sleep(0.002)

    except KeyboardInterrupt:
        print("\n🛑 Stopped.", flush=True)
    finally:
        try:
            ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
