#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
from collections import deque

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# =========================
# SETTINGS
# =========================
PORT = "COM5"
BAUD = 230400
SER_TIMEOUT = 0.05          # 讓 readline 不會卡太久
EXPECTED_MIN_FIELDS = 9     # 讀前9個：ax..mz

WINDOW = 1000               # 顯示最近1000點（約10秒@100Hz）
PLOT_UPDATE_MS = 50         # 圖更新頻率（20Hz更新圖，但資料仍照樣讀）

# 固定 y 範圍（不想固定就設成 None）
YLIM_ACCEL = (-1.5, 1.5)
YLIM_GYRO  = (-20, 20)
YLIM_MAG   = (-200, 200)


# =========================
# PARSER
# =========================
def parse_line_to_parts(line_bytes):
    try:
        s = line_bytes.decode("utf-8", errors="ignore").strip()
    except Exception:
        return None
    if not s:
        return None

    s = s.strip().strip("[]{}()")
    parts = [p.strip() for p in s.split(",")]
    if len(parts) < EXPECTED_MIN_FIELDS:
        return None
    return parts


# =========================
# READER THREAD
# =========================
class SerialIMUReader(threading.Thread):
    def __init__(self, ser, window=1000):
        super().__init__(daemon=True)
        self.ser = ser
        self.stop_event = threading.Event()

        self.i = 0
        self.x = deque(maxlen=window)

        self.ax = deque(maxlen=window)
        self.ay = deque(maxlen=window)
        self.az = deque(maxlen=window)
        self.gx = deque(maxlen=window)
        self.gy = deque(maxlen=window)
        self.gz = deque(maxlen=window)
        self.mx = deque(maxlen=window)
        self.my = deque(maxlen=window)
        self.mz = deque(maxlen=window)

        self.good = 0
        self.bad = 0

        self._last_hz_t = time.time()
        self._last_hz_n = 0
        self.hz = 0.0

    def run(self):
        # 清掉殘留資料
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

        while not self.stop_event.is_set():
            line = self.ser.readline()
            if not line:
                continue

            parts = parse_line_to_parts(line)
            if parts is None:
                self.bad += 1
                continue

            try:
                ax, ay, az, gx, gy, gz, mx, my, mz = map(float, parts[:9])
            except Exception:
                self.bad += 1
                continue

            self.i += 1
            self.x.append(self.i)

            self.ax.append(ax); self.ay.append(ay); self.az.append(az)
            self.gx.append(gx); self.gy.append(gy); self.gz.append(gz)
            self.mx.append(mx); self.my.append(my); self.mz.append(mz)

            self.good += 1

            # 粗略算一下接收 Hz（每秒更新一次）
            now = time.time()
            if now - self._last_hz_t >= 1.0:
                dn = self.good - self._last_hz_n
                self.hz = dn / (now - self._last_hz_t)
                self._last_hz_t = now
                self._last_hz_n = self.good

    def stop(self):
        self.stop_event.set()


# =========================
# MAIN
# =========================
def main():
    ser = serial.Serial(PORT, BAUD, timeout=SER_TIMEOUT)
    time.sleep(0.3)  # 有些板子開 port 會 reset，等一下更穩

    reader = SerialIMUReader(ser, window=WINDOW)
    reader.start()

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    ax1, ax2, ax3 = axes

    # Accel
    l_ax, = ax1.plot([], [], label="Ax")
    l_ay, = ax1.plot([], [], label="Ay")
    l_az, = ax1.plot([], [], label="Az")
    ax1.set_title("Acceleration")
    ax1.set_ylabel("g")
    ax1.grid(True)
    ax1.legend(loc="upper right")
    if YLIM_ACCEL is not None:
        ax1.set_ylim(-2, 2)  

    # Gyro
    l_gx, = ax2.plot([], [], label="Gx")
    l_gy, = ax2.plot([], [], label="Gy")
    l_gz, = ax2.plot([], [], label="Gz")
    ax2.set_title("Gyroscope")
    ax2.set_ylabel("rad/s")
    ax2.grid(True)
    ax2.legend(loc="upper right")
    if YLIM_GYRO is not None:
        ax2.set_ylim(-10, 10)

    # Mag
    l_mx, = ax3.plot([], [], label="Mx")
    l_my, = ax3.plot([], [], label="My")
    l_mz, = ax3.plot([], [], label="Mz")
    ax3.set_title("Magnetometer")
    ax3.set_ylabel("mGauss")
    ax3.set_xlabel("Sample")
    ax3.grid(True)
    ax3.legend(loc="upper right")
    if YLIM_MAG is not None:
        ax3.set_ylim(-1000, 1000)

    # 按 q 關閉
    def on_key(event):
        if event.key and event.key.lower() == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_):
        if len(reader.x) < 2:
            return (l_ax, l_ay, l_az, l_gx, l_gy, l_gz, l_mx, l_my, l_mz)

        x = list(reader.x)

        l_ax.set_data(x, list(reader.ax))
        l_ay.set_data(x, list(reader.ay))
        l_az.set_data(x, list(reader.az))

        l_gx.set_data(x, list(reader.gx))
        l_gy.set_data(x, list(reader.gy))
        l_gz.set_data(x, list(reader.gz))

        l_mx.set_data(x, list(reader.mx))
        l_my.set_data(x, list(reader.my))
        l_mz.set_data(x, list(reader.mz))

        ax3.set_xlim(x[0], x[-1])

        
        return (l_ax, l_ay, l_az, l_gx, l_gy, l_gz, l_mx, l_my, l_mz)

    anim = FuncAnimation(fig, update, interval=PLOT_UPDATE_MS, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

    # 關窗後收尾
    reader.stop()
    time.sleep(0.1)
    try:
        ser.close()
    except Exception:
        pass

    print(f"[INFO] Done. good={reader.good} bad={reader.bad}")

if __name__ == "__main__":
    main()
