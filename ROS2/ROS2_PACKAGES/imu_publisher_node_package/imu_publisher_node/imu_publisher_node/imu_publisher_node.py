#!/usr/bin/env python3
import serial
import time
import os
import sys
import math

import rclpy
from sensor_msgs.msg import Imu

PORT = "/dev/rfcomm0"
BAUDRATE = 230400
PRINT_HZ = 10.0
PRINT_PERIOD = 1.0 / PRINT_HZ

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
        # Debug: ver qué está llegando
        # print(f"[DEBUG] Line with {len(parts)} fields, skipping: {line}", flush=True)
        return None
    try:
        vals = list(map(float, parts))
        return dict(zip(FIELDS, vals))
    except ValueError:
        # print(f"[DEBUG] ValueError parsing line: {line}", flush=True)
        return None


def clear_screen():
    # más rápido que os.system("clear") y funciona bien en terminal
    print("\x1b[2J\x1b[H", end="", flush=True)


def build_imu_msg(node, s: dict) -> Imu:
    now = node.get_clock().now().to_msg()
    msg = Imu()
    msg.header.stamp = now
    msg.header.frame_id = "imu_link"

    msg.linear_acceleration.x = s["ax_g"] * G_TO_MS2
    msg.linear_acceleration.y = s["ay_g"] * G_TO_MS2
    msg.linear_acceleration.z = s["az_g"] * G_TO_MS2

    msg.angular_velocity.x = s["gx_dps"] * DEG_TO_RAD
    msg.angular_velocity.y = s["gy_dps"] * DEG_TO_RAD
    msg.angular_velocity.z = s["gz_dps"] * DEG_TO_RAD

    # Orientación identidad

    return msg


def main(args=None):
    # -------- ROS2 init --------
    rclpy.init(args=args)
    node = rclpy.create_node("simple_imu_publisher")
    imu_pub = node.create_publisher(Imu, "/imu/data", 10)

    # Forzar salida line-buffered para que los print se vean
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass

    print("✅ ROS2 node 'simple_imu_publisher' starting...", flush=True)

    # -------- Serial --------
    ser = connect_serial()
    rx_buf = ""
    last_sample = None
    last_print_t = time.monotonic()

    clear_screen()
    print("✅ Connected. Ctrl+C to stop.\n", flush=True)
    print("✅ Publishing Imu on /imu/data\n", flush=True)

    try:
        while rclpy.ok():
            # Procesar callbacks ROS
            rclpy.spin_once(node, timeout_sec=0.0)

            n = ser.in_waiting
            if n:
                chunk = ser.read(n).decode("utf-8", errors="ignore")
                rx_buf += chunk

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

                clear_screen()
                print("✅ ESP32 IMU (última muestra)  |  Ctrl+C para salir\n", flush=True)
                print(f"ACC [g]   ax={s['ax_g']:+8.3f}  ay={s['ay_g']:+8.3f}  az={s['az_g']:+8.3f}", flush=True)
                print(f"GYR [dps] gx={s['gx_dps']:+8.3f}  gy={s['gy_dps']:+8.3f}  gz={s['gz_dps']:+8.3f}", flush=True)
                print(f"MAG [uT]  mx={s['mx_uT']:+9.3f}  my={s['my_uT']:+9.3f}  mz={s['mz_uT']:+9.3f}", flush=True)
                print(f"P/T/Alt   P={s['p_hpa']:9.2f} hPa   T={s['t_C']:6.2f} °C   Alt={s['alt_m']:8.2f} m\n", flush=True)
                print(f"Print rate: {PRINT_HZ:.1f} Hz", flush=True)

                # Publicar en ROS
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
