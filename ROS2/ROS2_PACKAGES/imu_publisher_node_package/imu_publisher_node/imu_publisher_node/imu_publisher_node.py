#!/usr/bin/env python3
import serial
import time
import os
import sys
import math

import rclpy
from sensor_msgs.msg import Imu

# --- Madgwick + numpy for orientation estimation ---
try:
    import numpy as np
    from ahrs.filters import Madgwick
except ImportError as e:
    print(
        "ERROR: Required Python packages 'numpy' and 'ahrs' are not installed.\n"
        "Install them inside the container with:\n"
        "  pip3 install numpy ahrs\n",
        file=sys.stderr,
        flush=True,
    )
    raise

PORT = "/dev/rfcomm0"
BAUDRATE = 230400

PRINT_HZ = 10.0
PRINT_PERIOD = 1.0 / PRINT_HZ

# Frecuencia de muestreo aproximada del ESP32 (en Hz)
SAMPLING_RATE = 100.0

FIELDS = [
    "ax_g","ay_g","az_g",
    "gx_dps","gy_dps","gz_dps",
    "mx_uT","my_uT","mz_uT",
    "p_hpa","t_C","alt_m"
]

G_TO_MS2   = 9.80665
DEG_TO_RAD = math.pi / 180.0


def connect_serial():
    while True:
        try:
            ser = serial.Serial(PORT, BAUDRATE, timeout=0)
            print(f"✅ Connected to {PORT}", flush=True)
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
    print("\x1b[2J\x1b[H", end="", flush=True)


def build_imu_msg(node, s: dict, q=None) -> Imu:
    """Construye un sensor_msgs/Imu a partir de la muestra y cuaternión opcional q."""
    now = node.get_clock().now().to_msg()
    msg = Imu()
    msg.header.stamp = now
    msg.header.frame_id = "imu_link"

    # Aceleración lineal: g -> m/s^2
    msg.linear_acceleration.x = s["ax_g"] * G_TO_MS2
    msg.linear_acceleration.y = s["ay_g"] * G_TO_MS2
    msg.linear_acceleration.z = s["az_g"] * G_TO_MS2

    # Velocidad angular: deg/s -> rad/s
    msg.angular_velocity.x = s["gx_dps"] * DEG_TO_RAD
    msg.angular_velocity.y = s["gy_dps"] * DEG_TO_RAD
    msg.angular_velocity.z = s["gz_dps"] * DEG_TO_RAD

    # Orientación: si tenemos cuaternión válido, lo usamos
    if q is not None:
        try:
            msg.orientation.w = float(q[0])
            msg.orientation.x = float(q[1])
            msg.orientation.y = float(q[2])
            msg.orientation.z = float(q[3])
        except Exception:
            msg.orientation.w = 1.0
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
    else:
        # Identidad si algo falla
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0

    return msg


def main(args=None):
    # -------- ROS2 init --------
    rclpy.init(args=args)
    node = rclpy.create_node("simple_imu_publisher")
    imu_pub = node.create_publisher(Imu, "/imu/data", 10)

    # stdout line-buffered para que los print se vean
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass

    print("✅ ROS2 node 'simple_imu_publisher' starting with Madgwick IMU...", flush=True)

    # -------- Madgwick (solo IMU: acc + gyro) --------
    # Cuaternión inicial
    q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    # Filtro con frecuencia fija (no andamos cambiando frequency cada muestra)
    madgwick = Madgwick(beta=0.2, frequency=SAMPLING_RATE)

    def update_orientation(sample: dict):
        """Actualiza el cuaternión global q usando solo acelerómetro + giroscopio."""
        nonlocal q, madgwick

        ax = float(sample["ax_g"])
        ay = float(sample["ay_g"])
        az = float(sample["az_g"])

        gx_dps = float(sample["gx_dps"])
        gy_dps = float(sample["gy_dps"])
        gz_dps = float(sample["gz_dps"])

        # Vectores para el filtro
        acc = np.array([ax, ay, az], dtype=float)
        gyr = np.array([gx_dps, gy_dps, gz_dps], dtype=float) * DEG_TO_RAD

        try:
            new_q = madgwick.updateIMU(q, gyr=gyr, acc=acc)
            if new_q is not None:
                q = np.array(new_q, dtype=float)
        except Exception as e:
            print(f"[WARN] Madgwick IMU update error: {e}", flush=True)

    # -------- Serial --------
    ser = connect_serial()
    rx_buf = ""
    last_sample = None
    last_print_t = time.monotonic()

    clear_screen()
    print("✅ Connected. Ctrl+C to stop.\n", flush=True)
    print("✅ Publishing Imu on /imu/data (orientation from Madgwick IMU)\n", flush=True)

    try:
        while rclpy.ok():
            # Procesar callbacks ROS
            rclpy.spin_once(node, timeout_sec=0.0)

            n = ser.in_waiting
            if n:
                chunk = ser.read(n).decode("utf-8", errors="ignore")
                rx_buf += chunk

                # Procesar todas las líneas completas
                while "\n" in rx_buf:
                    line, rx_buf = rx_buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    sample = parse_csv12(line)
                    if sample is not None:
                        last_sample = sample
                        # Actualizar orientación con esta muestra
                        update_orientation(sample)

            now = time.monotonic()
            if last_sample is not None and (now - last_print_t) >= PRINT_PERIOD:
                last_print_t = now
                s = last_sample

                clear_screen()
                print("✅ ESP32 IMU (última muestra) :)  |  Ctrl+C para salir\n", flush=True)
                print(f"ACC [g]   ax={s['ax_g']:+8.3f}  ay={s['ay_g']:+8.3f}  az={s['az_g']:+8.3f}", flush=True)
                print(f"GYR [dps] gx={s['gx_dps']:+8.3f}  gy={s['gy_dps']:+8.3f}  gz={s['gz_dps']:+8.3f}", flush=True)
                print(f"MAG [uT]  mx={s['mx_uT']:+9.3f}  my={s['mx_uT']:+9.3f}  mz={s['mz_uT']:+9.3f}", flush=True)
                print(f"P/T/Alt   P={s['p_hpa']:9.2f} hPa   T={s['t_C']:6.2f} °C   Alt={s['alt_m']:8.2f} m\n", flush=True)
                print(f"Print rate: {PRINT_HZ:.1f} Hz", flush=True)

                # Publicar en ROS con la orientación actual q
                imu_msg = build_imu_msg(node, s, q=q)
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
