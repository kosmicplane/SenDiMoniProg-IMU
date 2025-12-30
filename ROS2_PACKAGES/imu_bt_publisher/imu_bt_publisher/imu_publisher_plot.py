#!/usr/bin/env python3
import serial
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

PORT = "/dev/rfcomm0"
BAUDRATE = 230400

PLOT_HZ = 20.0          # refresco de la UI (no tiene que ser igual a la IMU)
WINDOW_SEC = 10.0       # cuantos segundos se ven en la gráfica

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

    # Buffer circular para gráfica
    maxlen = int(WINDOW_SEC * PLOT_HZ) + 10
    t0 = time.monotonic()
    t = deque(maxlen=maxlen)

    ax_g = deque(maxlen=maxlen); ay_g = deque(maxlen=maxlen); az_g = deque(maxlen=maxlen)
    gx = deque(maxlen=maxlen);   gy = deque(maxlen=maxlen);   gz = deque(maxlen=maxlen)

    # --- Figure layout: panel texto arriba + 2 plots abajo ---
    fig = plt.figure()
    gs = fig.add_gridspec(3, 1, height_ratios=[1.1, 2.0, 2.0])

    ax_text = fig.add_subplot(gs[0])
    ax1 = fig.add_subplot(gs[1])
    ax2 = fig.add_subplot(gs[2], sharex=ax1)

    ax_text.axis("off")
    text_artist = ax_text.text(
        0.01, 0.5, "Esperando datos...",
        va="center", ha="left", fontsize=12, family="monospace"
    )

    # Líneas (sin fijar colores: deja default)
    l1, = ax1.plot([], [], label="ax_g")
    l2, = ax1.plot([], [], label="ay_g")
    l3, = ax1.plot([], [], label="az_g")
    ax1.set_ylabel("acc [g]")
    ax1.grid(True)
    ax1.legend(loc="upper right")

    g1, = ax2.plot([], [], label="gx_dps")
    g2, = ax2.plot([], [], label="gy_dps")
    g3, = ax2.plot([], [], label="gz_dps")
    ax2.set_ylabel("gyro [dps]")
    ax2.set_xlabel("time [s]")
    ax2.grid(True)
    ax2.legend(loc="upper right")

    fig.suptitle("ESP32 IMU — Live Plot + Dashboard", fontsize=14)

    def poll_serial():
        nonlocal rx_buf, last_sample
        try:
            n = ser.in_waiting
            if n:
                rx_buf += ser.read(n).decode("utf-8", errors="ignore")
                # procesa todas las líneas completas; conserva la última válida
                while "\n" in rx_buf:
                    line, rx_buf = rx_buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    sample = parse_csv12(line)
                    if sample is not None:
                        last_sample = sample
        except (serial.SerialException, OSError):
            # si se cae, deja last_sample como está y reintenta luego
            pass

    def update(_frame):
        nonlocal last_sample, ser

        # lee todo lo disponible y quédate con lo último
        poll_serial()

        # Si no hay nada aún, solo actualiza texto
        if last_sample is None:
            text_artist.set_text("Esperando datos... (¿tu ESP32 está enviando líneas con \\n?)")
            return (text_artist, l1, l2, l3, g1, g2, g3)

        # tiempo actual
        now = time.monotonic() - t0

        s = last_sample
        t.append(now)
        ax_g.append(s["ax_g"]); ay_g.append(s["ay_g"]); az_g.append(s["az_g"])
        gx.append(s["gx_dps"]); gy.append(s["gy_dps"]); gz.append(s["gz_dps"])

        # Panel texto (dashboard)
        dash = (
            f"ACC[g]   ax={s['ax_g']:+8.3f}  ay={s['ay_g']:+8.3f}  az={s['az_g']:+8.3f}\n"
            f"GYR[dps] gx={s['gx_dps']:+8.3f}  gy={s['gy_dps']:+8.3f}  gz={s['gz_dps']:+8.3f}\n"
            f"MAG[uT]  mx={s['mx_uT']:+9.3f} my={s['my_uT']:+9.3f} mz={s['mz_uT']:+9.3f}\n"
            f"P/T/Alt  P={s['p_hpa']:9.2f} hPa   T={s['t_C']:6.2f} °C   Alt={s['alt_m']:8.2f} m\n"
            f"UI: {PLOT_HZ:.1f} Hz   Window: {WINDOW_SEC:.1f} s   Port: {PORT}"
        )
        text_artist.set_text(dash)

        # set data líneas
        l1.set_data(t, ax_g); l2.set_data(t, ay_g); l3.set_data(t, az_g)
        g1.set_data(t, gx);   g2.set_data(t, gy);   g3.set_data(t, gz)

        # ventana deslizante en X
        if len(t) > 2:
            ax1.set_xlim(max(0.0, t[-1] - WINDOW_SEC), t[-1])

        # autoscale Y
        ax1.relim(); ax1.autoscale_view(scalex=False, scaley=True)
        ax2.relim(); ax2.autoscale_view(scalex=False, scaley=True)

        return (text_artist, l1, l2, l3, g1, g2, g3)

    interval_ms = int(1000.0 / PLOT_HZ)
    ani = FuncAnimation(fig, update, interval=interval_ms, blit=False)

    try:
        plt.show()
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
