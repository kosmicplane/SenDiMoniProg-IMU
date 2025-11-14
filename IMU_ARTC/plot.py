#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
即時顯示 IMU 九軸 + 海拔(Altitude)：
- 子圖1：ax, ay, az（g）
- 子圖2：gx, gy, gz（°/s 或 rad/s，依你的輸出單位）
- 子圖3：mx, my, mz（uT 或無單位）
- 子圖4：altitude（m；來自輸出第 12 欄）

重點：
- 背景執行緒讀取序列埠，主執行緒只負責繪圖（更順）
- X 軸固定顯示最近 WINDOW_SEC 秒，會跟著時間滾動
- 抽點 decimation，避免每幀重繪過多點
- blit 加速重繪
- 容錯解析：壞行自動略過

假設每行資料格式：
ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temperature,altitude
"""

import threading
import time
from collections import deque

import numpy as np
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ======== 參數 ========
PORT = "/dev/ttyUSB0"    # Linux：/dev/ttyUSB0 或 /dev/ttyACM0
BAUD = 230400
TIMEOUT = 0.02           # 短逾時，降低卡頓
WINDOW_SEC = 30          # X 軸固定顯示最近 N 秒
PLOT_MAX_POINTS = 1500   # 單曲線最多繪製點數（自動抽點）
READ_BUFFER_MAX = 60000  # 背景緩衝上限
# ======================

# 背景緩衝：放 (t, ax, ay, az, gx, gy, gz, mx, my, mz, alt)
buf = deque(maxlen=READ_BUFFER_MAX)
buf_lock = threading.Lock()
run_flag = True
t0 = time.monotonic()

def parse_line(s: str):
    """
    解析一行 CSV -> 回傳 11 欄：
    (ax, ay, az, gx, gy, gz, mx, my, mz, alt, ok)
    若資料不足或格式錯誤，回傳 (..., False)
    """
    try:
        parts = s.strip().split(",")
        if len(parts) < 12:
            return (None,)*10 + (False,)
        ax = float(parts[0]); ay = float(parts[1]); az = float(parts[2])
        gx = float(parts[3]); gy = float(parts[4]); gz = float(parts[5])
        mx = float(parts[6]); my = float(parts[7]); mz = float(parts[8])
        alt = float(parts[11])  # altitude 在第 12 欄
        return ax, ay, az, gx, gy, gz, mx, my, mz, alt, True
    except Exception:
        return (None,)*10 + (False,)

def reader_thread():
    """ 序列讀取執行緒：快速讀 -> 解析 -> 丟進共享緩衝 """
    global run_flag
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    except serial.SerialException as e:
        print("[ERROR] 開啟序列埠失敗：", e)
        run_flag = False
        return
    with ser:
        while run_flag:
            raw = ser.readline()
            if not raw:
                continue
            try:
                s = raw.decode("utf-8", errors="ignore")
            except Exception:
                continue
            ax, ay, az, gx, gy, gz, mx, my, mz, alt, ok = parse_line(s)
            if not ok:
                continue
            t = time.monotonic() - t0
            with buf_lock:
                buf.append((t, ax, ay, az, gx, gy, gz, mx, my, mz, alt))

# 啟動背景讀取
th = threading.Thread(target=reader_thread, daemon=True)
th.start()

# ======== 視覺化 ========
plt.rcParams['path.simplify'] = True
plt.rcParams['path.simplify_threshold'] = 1.0

fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
ax_acc, ax_gyr, ax_mag, ax_alt = axes
fig.suptitle("Live IMU (ax/ay/az, gx/gy/gz, mx/my/mz, altitude)")

# 加速度
la_x, = ax_acc.plot([], [], lw=1.2, label="ax (g)", animated=True)
la_y, = ax_acc.plot([], [], lw=1.2, label="ay (g)", animated=True)
la_z, = ax_acc.plot([], [], lw=1.2, label="az (g)", animated=True)
ax_acc.set_ylabel("Accel (g)")
ax_acc.grid(True)
ax_acc.legend(loc="upper right")

# 陀螺儀
lg_x, = ax_gyr.plot([], [], lw=1.2, label="gx", animated=True)
lg_y, = ax_gyr.plot([], [], lw=1.2, label="gy", animated=True)
lg_z, = ax_gyr.plot([], [], lw=1.2, label="gz", animated=True)
ax_gyr.set_ylabel("Gyro")
ax_gyr.grid(True)
ax_gyr.legend(loc="upper right")

# 磁力計
lm_x, = ax_mag.plot([], [], lw=1.2, label="mx", animated=True)
lm_y, = ax_mag.plot([], [], lw=1.2, label="my", animated=True)
lm_z, = ax_mag.plot([], [], lw=1.2, label="mz", animated=True)
ax_mag.set_ylabel("Mag")
ax_mag.grid(True)
ax_mag.legend(loc="upper right")

# 海拔
l_alt, = ax_alt.plot([], [], lw=1.4, label="altitude (m)", animated=True)
ax_alt.set_xlabel("Time (s)")
ax_alt.set_ylabel("Alt (m)")
ax_alt.grid(True)
ax_alt.legend(loc="upper right")

# 佇列保存目前窗口資料
times = deque()
axs = deque(); ays = deque(); azs = deque()
gxs = deque(); gys = deque(); gzs = deque()
mxs = deque(); mys = deque(); mzs = deque()
alts = deque()

def pop_all_from_buf():
    with buf_lock:
        if not buf:
            return []
        out = list(buf)
        buf.clear()
        return out

def decimate(xs, ys, max_points):
    """ 抽點（簡單步進抽樣），避免重繪太多點 """
    n = len(xs)
    if n <= max_points:
        return xs, ys
    step = n // max_points + 1
    return xs[::step], ys[::step]

# 初次繪製與背景快照（blit 需要）
fig.canvas.draw()
bg = fig.canvas.copy_from_bbox(fig.bbox)
prev_xlim = (0.0, WINDOW_SEC)

def refresh_bg():
    """ 尺寸或座標更動時需重抓背景 """
    fig.canvas.draw()
    return fig.canvas.copy_from_bbox(fig.bbox)

def trim_window():
    """ 丟掉視窗外的資料，只保留最近 WINDOW_SEC 秒 """
    if not times:
        return
    tmax = times[-1]
    tmin = tmax - WINDOW_SEC
    while times and times[0] < tmin:
        times.popleft(); axs.popleft(); ays.popleft(); azs.popleft()
        gxs.popleft();  gys.popleft();  gzs.popleft()
        mxs.popleft();  mys.popleft();  mzs.popleft()
        alts.popleft()

def update(_):
    global bg, prev_xlim

    # 拿走背景緩衝的所有新資料
    new = pop_all_from_buf()
    if new:
        for t, ax_v, ay_v, az_v, gx_v, gy_v, gz_v, mx_v, my_v, mz_v, alt_v in new:
            times.append(t)
            axs.append(ax_v); ays.append(ay_v); azs.append(az_v)
            gxs.append(gx_v); gys.append(gy_v); gzs.append(gz_v)
            mxs.append(mx_v); mys.append(my_v); mzs.append(mz_v)
            alts.append(alt_v)
        trim_window()

    if not times:
        return (la_x, la_y, la_z,
                lg_x, lg_y, lg_z,
                lm_x, lm_y, lm_z,
                l_alt)

    # 轉 numpy
    x  = np.fromiter(times, dtype=float)
    ax = np.fromiter(axs, dtype=float)
    ay = np.fromiter(ays, dtype=float)
    az = np.fromiter(azs, dtype=float)
    gx = np.fromiter(gxs, dtype=float)
    gy = np.fromiter(gys, dtype=float)
    gz = np.fromiter(gzs, dtype=float)
    mx = np.fromiter(mxs, dtype=float)
    my = np.fromiter(mys, dtype=float)
    mz = np.fromiter(mzs, dtype=float)
    alt = np.fromiter(alts, dtype=float)

    # 抽點
    x_d, ax_d = decimate(x, ax, PLOT_MAX_POINTS)
    _,   ay_d = decimate(x, ay, PLOT_MAX_POINTS)
    _,   az_d = decimate(x, az, PLOT_MAX_POINTS)

    _,   gx_d = decimate(x, gx, PLOT_MAX_POINTS)
    _,   gy_d = decimate(x, gy, PLOT_MAX_POINTS)
    _,   gz_d = decimate(x, gz, PLOT_MAX_POINTS)

    _,   mx_d = decimate(x, mx, PLOT_MAX_POINTS)
    _,   my_d = decimate(x, my, PLOT_MAX_POINTS)
    _,   mz_d = decimate(x, mz, PLOT_MAX_POINTS)

    _,   alt_d = decimate(x, alt, PLOT_MAX_POINTS)

    # X 軸：固定顯示 [tmax - WINDOW_SEC, tmax]
    tmax = x[-1]
    xmin = max(0.0, tmax - WINDOW_SEC)
    xmax = max(xmin + 1e-3, tmax)
    need_xlim_update = (prev_xlim[0] != xmin) or (prev_xlim[1] != xmax)
    if need_xlim_update:
        for axh in (ax_acc, ax_gyr, ax_mag, ax_alt):
            axh.set_xlim(xmin, xmax)
        bg = refresh_bg()
        prev_xlim = (xmin, xmax)

    # Y 軸自動範圍
    def set_ylim_auto(axh, *ys):
        ymin = float(min([y.min() if len(y) else 0.0 for y in ys]))
        ymax = float(max([y.max() if len(y) else 0.0 for y in ys]))
        if ymin == ymax:
            ymin, ymax = ymin - 1.0, ymax + 1.0
        margin = 0.1 * (ymax - ymin)
        axh.set_ylim(ymin - margin, ymax + margin)

    set_ylim_auto(ax_acc, ax_d, ay_d, az_d)
    set_ylim_auto(ax_gyr, gx_d, gy_d, gz_d)
    set_ylim_auto(ax_mag, mx_d, my_d, mz_d)
    set_ylim_auto(ax_alt, alt_d)

    # 還原背景後只畫線（blit）
    fig.canvas.restore_region(bg)

    la_x.set_data(x_d, ax_d)
    la_y.set_data(x_d, ay_d)
    la_z.set_data(x_d, az_d)
    ax_acc.draw_artist(la_x); ax_acc.draw_artist(la_y); ax_acc.draw_artist(la_z)

    lg_x.set_data(x_d, gx_d)
    lg_y.set_data(x_d, gy_d)
    lg_z.set_data(x_d, gz_d)
    ax_gyr.draw_artist(lg_x); ax_gyr.draw_artist(lg_y); ax_gyr.draw_artist(lg_z)

    lm_x.set_data(x_d, mx_d)
    lm_y.set_data(x_d, my_d)
    lm_z.set_data(x_d, mz_d)
    ax_mag.draw_artist(lm_x); ax_mag.draw_artist(lm_y); ax_mag.draw_artist(lm_z)

    l_alt.set_data(x_d, alt_d)
    ax_alt.draw_artist(l_alt)

    fig.canvas.blit(fig.bbox)
    fig.canvas.flush_events()

    return (la_x, la_y, la_z,
            lg_x, lg_y, lg_z,
            lm_x, lm_y, lm_z,
            l_alt)

def on_resize(_evt):
    # 視窗大小改變也要重抓背景
    global bg
    bg = refresh_bg()

def on_close(_evt):
    global run_flag
    run_flag = False

fig.canvas.mpl_connect("resize_event", on_resize)
fig.canvas.mpl_connect("close_event", on_close)

ani = animation.FuncAnimation(fig, update, interval=33, blit=True)
plt.tight_layout()
plt.show()

run_flag = False
th.join(timeout=0.5)
