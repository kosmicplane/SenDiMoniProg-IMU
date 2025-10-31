import time
import serial
import matplotlib
matplotlib.use("TkAgg")  # 很重要：避免某些環境互動視窗卡死
import matplotlib.pyplot as plt
from collections import deque

# ========================
# 使用者參數
# ========================
PORT = "/dev/ttyUSB0"   # 改成正確的port，例如 /dev/ttyUSB1, /dev/ttyACM0, COM5...
BAUD = 230400
TIMEOUT = 1.0
WINDOW_SIZE = 200

# ========================
# 緩衝區
# ========================
t_buf   = deque(maxlen=WINDOW_SIZE)
ax_buf  = deque(maxlen=WINDOW_SIZE)
ay_buf  = deque(maxlen=WINDOW_SIZE)
az_buf  = deque(maxlen=WINDOW_SIZE)

gx_buf  = deque(maxlen=WINDOW_SIZE)
gy_buf  = deque(maxlen=WINDOW_SIZE)
gz_buf  = deque(maxlen=WINDOW_SIZE)

mx_buf  = deque(maxlen=WINDOW_SIZE)
my_buf  = deque(maxlen=WINDOW_SIZE)
mz_buf  = deque(maxlen=WINDOW_SIZE)

alt_buf = deque(maxlen=WINDOW_SIZE)

t0 = None  # 開始時間 (第一筆資料時間)
last_print_time = 0  # 用來降低print頻率，避免洗螢幕太快


def parse_line(line):
    """
    嘗試解析你的 IMU 字串。
    預期格式:
    A[g]:ax,ay,az|G[rad/s]:gx,gy,gz|M[mG]:mx,my,mz|...|...|Alt[m]:alt
    回傳 tuple 或 None
    """
    line = line.strip()
    if not line.startswith("A[g]:"):
        return None

    parts = line.split("|")
    if len(parts) < 6:
        return None

    try:
        a_vals = parts[0].replace("A[g]:", "").split(",")
        ax = float(a_vals[0])
        ay = float(a_vals[1])
        az = float(a_vals[2])

        g_vals = parts[1].replace("G[rad/s]:", "").split(",")
        gx = float(g_vals[0])
        gy = float(g_vals[1])
        gz = float(g_vals[2])

        m_vals = parts[2].replace("M[mG]:", "").split(",")
        mx = float(m_vals[0])
        my = float(m_vals[1])
        mz = float(m_vals[2])

        alt_str = parts[5].replace("Alt[m]:", "").strip()
        alt = float(alt_str)

        return ax, ay, az, gx, gy, gz, mx, my, mz, alt

    except Exception as e:
        print("[PARSE EXCEPTION]", e)
        return None


def autoscale_axis(ax, lists, pad_ratio=0.1, fallback=(-1, 1)):
    vals = []
    for arr in lists:
        vals.extend(arr)
    if not vals:
        ax.set_ylim(fallback[0], fallback[1])
        return
    vmin = min(vals)
    vmax = max(vals)
    if vmin == vmax:
        ax.set_ylim(vmin - 1, vmax + 1)
    else:
        pad = (vmax - vmin) * pad_ratio
        ax.set_ylim(vmin - pad, vmax + pad)


# === Matplotlib figure ===
plt.ion()
fig, axes = plt.subplots(4, 1, figsize=(10, 10))
fig.canvas.manager.set_window_title("IMU Debug Real-Time Plot")

ax_acc = axes[0]
line_ax, = ax_acc.plot([0], [0], label="Ax")
line_ay, = ax_acc.plot([0], [0], label="Ay")
line_az, = ax_acc.plot([0], [0], label="Az")
ax_acc.set_ylabel("Accel (g)")
ax_acc.set_title("Accelerometer")
ax_acc.grid(True)
ax_acc.legend(loc="upper left")

ax_gyro = axes[1]
line_gx, = ax_gyro.plot([0], [0], label="Gx")
line_gy, = ax_gyro.plot([0], [0], label="Gy")
line_gz, = ax_gyro.plot([0], [0], label="Gz")
ax_gyro.set_ylabel("Gyro (rad/s)")
ax_gyro.set_title("Gyroscope")
ax_gyro.grid(True)
ax_gyro.legend(loc="upper left")

ax_mag = axes[2]
line_mx, = ax_mag.plot([0], [0], label="Mx")
line_my, = ax_mag.plot([0], [0], label="My")
line_mz, = ax_mag.plot([0], [0], label="Mz")
ax_mag.set_ylabel("Mag (mG)")
ax_mag.set_title("Magnetometer")
ax_mag.grid(True)
ax_mag.legend(loc="upper left")

ax_alt = axes[3]
line_alt, = ax_alt.plot([0], [0], label="Alt")
ax_alt.set_ylabel("Alt (m)")
ax_alt.set_xlabel("Time (s)")
ax_alt.set_title("Altitude")
ax_alt.grid(True)
ax_alt.legend(loc="upper left")

# === Serial open ===
print(f"[MAIN] Opening serial {PORT} @ {BAUD} ...")
ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
time.sleep(2.0)
print("[MAIN] Serial opened.")

try:
    while True:
        raw = ser.readline().decode("utf-8", errors="ignore").strip()

        now_sys = time.time()

        if raw == "":
            # 沒讀到任何資料（timeout)
            # 我們每0.5秒提醒一次
            if now_sys - last_print_time > 0.5:
                print("[DEBUG] no data...")
                last_print_time = now_sys

            plt.pause(0.001)
            continue

        # 印出原始字串 (低頻率)
        if now_sys - last_print_time > 0.5:
            print("[RAW]", raw)
            last_print_time = now_sys

        parsed = parse_line(raw)

        if parsed is None:
            # 格式不符合
            # 我們標記失敗一次，幫你知道是不是parser問題
            print("[PARSE FAIL]")
            plt.pause(0.001)
            continue
        else:
            # 有成功解析資料
            ax_val, ay_val, az_val, gx_val, gy_val, gz_val, mx_val, my_val, mz_val, alt_val = parsed
            # 只偶爾印（避免洗爆）
            # 這行會讓你確認數值是不是固定一樣
            print("[PARSED OK] ax=", ax_val, "ay=", ay_val, "alt=", alt_val)

        # 時間戳 (t=0 為第一筆成功解析的資料時間)
        if t0 is None:
            t0 = now_sys
        t_now = now_sys - t0

        # push data
        t_buf.append(t_now)
        ax_buf.append(ax_val); ay_buf.append(ay_val); az_buf.append(az_val)
        gx_buf.append(gx_val); gy_buf.append(gy_val); gz_buf.append(gz_val)
        mx_buf.append(mx_val); my_buf.append(my_val); mz_buf.append(mz_val)
        alt_buf.append(alt_val)

        # turn deque -> list for plotting
        t_list   = list(t_buf)
        ax_list  = list(ax_buf)
        ay_list  = list(ay_buf)
        az_list  = list(az_buf)
        gx_list  = list(gx_buf)
        gy_list  = list(gy_buf)
        gz_list  = list(gz_buf)
        mx_list  = list(mx_buf)
        my_list  = list(my_buf)
        mz_list  = list(mz_buf)
        alt_list = list(alt_buf)

        # update each line's x and y
        line_ax.set_xdata(t_list); line_ax.set_ydata(ax_list)
        line_ay.set_xdata(t_list); line_ay.set_ydata(ay_list)
        line_az.set_xdata(t_list); line_az.set_ydata(az_list)

        line_gx.set_xdata(t_list); line_gx.set_ydata(gx_list)
        line_gy.set_xdata(t_list); line_gy.set_ydata(gy_list)
        line_gz.set_xdata(t_list); line_gz.set_ydata(gz_list)

        line_mx.set_xdata(t_list); line_mx.set_ydata(mx_list)
        line_my.set_xdata(t_list); line_my.set_ydata(my_list)
        line_mz.set_xdata(t_list); line_mz.set_ydata(mz_list)

        line_alt.set_xdata(t_list); line_alt.set_ydata(alt_list)

        # x 軸範圍 = 目前視窗
        if len(t_list) >= 2:
            t_min = t_list[0]
            t_max = t_list[-1]
            ax_acc.set_xlim(t_min, t_max)
            ax_gyro.set_xlim(t_min, t_max)
            ax_mag.set_xlim(t_min, t_max)
            ax_alt.set_xlim(t_min, t_max)

        # y 軸自動縮放
        autoscale_axis(ax_acc, [ax_buf, ay_buf, az_buf], fallback=(-5, 5))
        autoscale_axis(ax_gyro, [gx_buf, gy_buf, gz_buf], fallback=(-10, 10))
        autoscale_axis(ax_mag, [mx_buf, my_buf, mz_buf], fallback=(-1000, 1000))
        autoscale_axis(ax_alt, [alt_buf], fallback=(0, 2))

        # redraw
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.001)

except KeyboardInterrupt:
    print("[MAIN] Ctrl+C stop")

finally:
    ser.close()
    plt.ioff()
    plt.show()
    print("[MAIN] done")
