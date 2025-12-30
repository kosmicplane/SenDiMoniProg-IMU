#!/usr/bin/env python3
import serial
import time
import math

PORT = "/dev/rfcomm0"
BAUDRATE = 230400

# Ajusta esto: 10 Hz = imprime cada 0.1s
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
            ser = serial.Serial(PORT, BAUDRATE, timeout=0)  # no bloqueante
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

def main():
    ser = connect_serial()

    rx_buf = ""
    last_sample = None
    last_print_t = time.monotonic()

    try:
        while True:
            # Leer lo disponible (para no quedarnos atrás)
            n = ser.in_waiting
            if n:
                rx_buf += ser.read(n).decode("utf-8", errors="ignore")

                # Procesar todas las líneas completas, quedarnos con la última válida
                while "\n" in rx_buf:
                    line, rx_buf = rx_buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    sample = parse_csv12(line)
                    if sample is not None:
                        last_sample = sample

            # Imprimir a tasa fija (throttle)
            now = time.monotonic()
            if (now - last_print_t) >= PRINT_PERIOD and last_sample is not None:
                last_print_t = now

                s = last_sample
                # Formato organizado (ajusta decimales a gusto)
                out = (
                    f"ACC[g]:  ax={s['ax_g']:+7.3f}  ay={s['ay_g']:+7.3f}  az={s['az_g']:+7.3f} | "
                    f"GYR[dps]: gx={s['gx_dps']:+8.3f} gy={s['gy_dps']:+8.3f} gz={s['gz_dps']:+8.3f} | "
                    f"MAG[uT]:  mx={s['mx_uT']:+9.3f} my={s['my_uT']:+9.3f} mz={s['mz_uT']:+9.3f} | "
                    f"P={s['p_hpa']:8.2f} hPa  T={s['t_C']:6.2f} C  Alt={s['alt_m']:7.2f} m"
                )
                print(out, flush=True)

            # Pequeño sleep para no usar 100% CPU
            if n == 0:
                time.sleep(0.002)

    except KeyboardInterrupt:
        print("\n🛑 Stopped.", flush=True)
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
