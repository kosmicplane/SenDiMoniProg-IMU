#!/usr/bin/env python3
import serial
import time
import os

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

def main():
    ser = connect_serial()
    rx_buf = ""
    last_sample = None
    last_print_t = time.monotonic()

    clear_screen()
    print("✅ Connected. Ctrl+C to stop.\n", flush=True)

    try:
        while True:
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

                clear_screen()
                print("✅ ESP32 IMU (última muestra)  |  Ctrl+C para salir\n")
                print(f"ACC [g]   ax={s['ax_g']:+8.3f}  ay={s['ay_g']:+8.3f}  az={s['az_g']:+8.3f}")
                print(f"GYR [dps] gx={s['gx_dps']:+8.3f}  gy={s['gy_dps']:+8.3f}  gz={s['gz_dps']:+8.3f}")
                print(f"MAG [uT]  mx={s['mx_uT']:+9.3f}  my={s['my_uT']:+9.3f}  mz={s['mz_uT']:+9.3f}")
                print(f"P/T/Alt   P={s['p_hpa']:9.2f} hPa   T={s['t_C']:6.2f} °C   Alt={s['alt_m']:8.2f} m\n")
                print(f"Print rate: {PRINT_HZ:.1f} Hz")

            if n == 0:
                time.sleep(0.002)

    except KeyboardInterrupt:
        print("\n🛑 Stopped.", flush=True)
    finally:
        try: ser.close()
        except: pass

if __name__ == "__main__":
    main()
