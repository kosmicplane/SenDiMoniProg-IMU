#!/usr/bin/env python3
import serial
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

PORT = "/dev/rfcomm0"
BAUDRATE = 230400

PLOT_HZ = 20.0
DT_PLOT = 1.0 / PLOT_HZ
WINDOW_SEC = 10.0

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

    maxlen = int(WINDOW_SEC * PLOT_HZ) + 5
    t0 = time.monotonic()
    t = deque(maxlen=maxlen)

    ax_g = deque(maxlen=maxlen); ay_g = deque(maxlen=maxlen); az_g = deque(maxlen=maxlen)
    gx = deque(maxlen=maxlen);   gy = deque(maxlen=maxlen);   gz = deque(maxlen=maxlen)

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    l1, = ax1.plot([], [], label="ax_g")
    l2, = ax1.plot([], [], label="ay_g")
    l3, = ax1.plot([], [], label="az_g")
    ax1.set_ylabel("acc [g]")
    ax1.legend(loc="upper right")
    ax1.grid(True)

    g1, = ax2.plot([], [], label="gx_dps")
    g2, = ax2.plot([], [], label="gy_dps")
    g3, = ax2.plot([], [], label="gz_dps")
    ax2.set_ylabel("gyro [dps]")
    ax2.set_xlabel("time [s]")
    ax2.legend(loc="upper right")
    ax2.grid(True)

    def poll_serial():
        nonlocal rx_buf, last_sample
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

    def update(_frame):
        # lee todo lo disponible y quédate con la última muestra
        poll_serial()
        if last_sample is None:
            return (l1, l2, l3, g1, g2, g3)

        now = time.monotonic() - t0
        s = last_sample

        t.append(now)
        ax_g.append(s["ax_g"]); ay_g.append(s["ay_g"]); az_g.append(s["az_g"])
        gx.append(s["gx_dps"]); gy.append(s["gy_dps"]); gz.append(s["gz_dps"])

        # set data
        l1.set_data(t, ax_g); l2.set_data(t, ay_g); l3.set_data(t, az_g)
        g1.set_data(t, gx);   g2.set_data(t, gy);   g3.set_data(t, gz)

        # ventana deslizante
        if len(t) > 2:
            ax1.set_xlim(max(0.0, t[-1] - WINDOW_SEC), t[-1])

        # autoscale Y suavemente
        ax1.relim(); ax1.autoscale_view(scalex=False, scaley=True)
        ax2.relim(); ax2.autoscale_view(scalex=False, scaley=True)

        return (l1, l2, l3, g1, g2, g3)

    ani = FuncAnimation(fig, update, interval=DT_PLOT * 1000, blit=False)
    try:
        plt.show()
    finally:
        try: ser.close()
        except: pass

if __name__ == "__main__":
    main()
